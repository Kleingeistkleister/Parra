import logging
import threading
import os
import time
import serial
import msgproto
import chelper
import util

uart_timeout = 0.5
start_time = 0

def timeout():
    global start_time
    start_time = time.monotonic()

class error(Exception):
    pass

class SerialReader:
    def __init__(self, warn_prefix=""):
        self.warn_prefix = warn_prefix
        # Serial port
        self.serial_dev = None
        self.msgparser = msgproto.MessageParser(warn_prefix=warn_prefix)
        
        # C interface
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.serialqueue = None
        self.default_cmd_queue = self.alloc_command_queue()
        self.stats_buf = self.ffi_main.new('char[4096]')
        
        # Threading
        self.lock = threading.Lock()
        self.background_thread = None
        
        # Message handlers
        self.handlers = {}
        self.register_response(self._handle_unknown_init, '#unknown')
        self.register_response(self.handle_output, '#output')
        
        # Sent message notification tracking
        self.last_notify_id = 0
        self.pending_notifications = {}
    
    def _error(self, msg, *params):
        raise error(self.warn_prefix + (msg % params))

    def _get_identify_data(self):
        # Query the "data dictionary" from the micro-controller
        identify_data = b""
        while True:
            msg = "identify offset=%d count=%d" % (len(identify_data), 40)
            try:
                params = self.send_with_response(msg, 'identify_response')
            except error as e:
                logging.exception("%sWait for identify_response", self.warn_prefix)
                return None
            if params['offset'] == len(identify_data):
                msgdata = params['data']
                if not msgdata:
                    # Done
                    return identify_data
                identify_data += msgdata
    
    def _start_session(self, serial_dev, serial_fd_type=b'u', client_id=0):
        self.serial_dev = serial_dev
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(serial_dev.fileno(), serial_fd_type, client_id),
            self.ffi_lib.serialqueue_free)
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start()
        
        # Obtain and load the data dictionary from the firmware
        identify_data = self._get_identify_data()
        
        if identify_data is None:
            logging.info("%sTimeout on connect", self.warn_prefix)
            self.disconnect()
            return False
        
        msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
        msgparser.process_identify(identify_data)
        
        self.msgparser = msgparser
        self.register_response(self.handle_unknown, '#unknown')
        
        # Setup baud adjust
        if serial_fd_type == b'c':
            wire_freq = msgparser.get_constant_float('CANBUS_FREQUENCY', None)
            
        else:
            wire_freq = msgparser.get_constant_float('SERIAL_BAUD', None)
            
        if wire_freq is not None:
            self.ffi_lib.serialqueue_set_wire_frequency(self.serialqueue, wire_freq)
            
        receive_window = msgparser.get_constant_int('RECEIVE_WINDOW', None)
        
        if receive_window is not None:
            self.ffi_lib.serialqueue_set_receive_window(
                self.serialqueue, receive_window)
        return True
    
    def connect_uart(self, serialport, baud, rts=True):
        # Initial connection
        logging.info("%sStarting serial connect", self.warn_prefix)
        timeout()
        while True:
            if time.monotonic() > start_time + 90.:
                self._error("Unable to connect")
            try:
                serial_dev = serial.Serial(baudrate=baud, timeout=0,
                                           exclusive=True)
                serial_dev.port = serialport
                serial_dev.rts = rts
                serial_dev.open()
            except (OSError, IOError, serial.SerialException) as e:
                logging.warning("%sUnable to open serial port: %s", self.warn_prefix, e)
                time.sleep(5)
                continue
            stk500v2_leave(serial_dev)
            ret = self._start_session(serial_dev)
            if ret:
                break
    
    def connect_file(self, debugoutput, dictionary, pace=False):
        self.serial_dev = debugoutput
        self.msgparser.process_identify(dictionary, decompress=False)
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(self.serial_dev.fileno(), b'f', 0),
            self.ffi_lib.serialqueue_free)
    
    def set_clock_est(self, freq, conv_time, conv_clock, last_clock):
        self.ffi_lib.serialqueue_set_clock_est(
            self.serialqueue, freq, conv_time, conv_clock, last_clock)
    
    def disconnect(self):
        if self.serialqueue is not None:
            self.ffi_lib.serialqueue_exit(self.serialqueue)
            if self.background_thread is not None:
                self.background_thread.join()
            self.background_thread = self.serialqueue = None
        if self.serial_dev is not None:
            self.serial_dev.close()
            self.serial_dev = None
        for pn in self.pending_notifications.values():
            pn.complete(None)
        self.pending_notifications.clear()
    
    def stats(self):
        if self.serialqueue is None:
            return ""
        self.ffi_lib.serialqueue_get_stats(self.serialqueue,
                                           self.stats_buf, len(self.stats_buf))
        return str(self.ffi_main.string(self.stats_buf).decode())
    
    def get_msgparser(self):
        return self.msgparser
    
    def get_serialqueue(self):
        return self.serialqueue
    
    def get_default_command_queue(self):
        return self.default_cmd_queue
    
    # Serial response callbacks
    def register_response(self, callback, name, oid=None):
        with self.lock:
            if callback is None:
                del self.handlers[name, oid]
            else:
                self.handlers[name, oid] = callback
    
    # Command sending
    def raw_send(self, cmd, minclock, reqclock, cmd_queue):
        self.ffi_lib.serialqueue_send(self.serialqueue, cmd_queue, cmd, len(cmd), minclock, reqclock, 0)
    
    def raw_send_wait_ack(self, cmd, minclock, reqclock, cmd_queue):
        self.last_notify_id += 1
        nid = self.last_notify_id
        completion = threading.Event()
        self.pending_notifications[nid] = completion
        self.ffi_lib.serialqueue_send(self.serialqueue, cmd_queue,
                                      cmd, len(cmd), minclock, reqclock, nid)
        completion.wait(timeout=uart_timeout)
        if nid in self.pending_notifications:
            del self.pending_notifications[nid]
            self._error("Serial connection closed")
    
    def send(self, msg, minclock=0, reqclock=0):
        cmd = self.msgparser.create_command(msg)
        self.raw_send(cmd, minclock, reqclock, self.default_cmd_queue)
    
    def send_with_response(self, msg, response):
        cmd = self.msgparser.create_command(msg)
        src = SerialRetryCommand(self, response)
        return src.get_response([cmd], self.default_cmd_queue)
    
    def alloc_command_queue(self):
        return self.ffi_main.gc(self.ffi_lib.serialqueue_alloc_commandqueue(), self.ffi_lib.serialqueue_free_commandqueue)
    
    # Dumping debug lists
    def dump_debug(self):
        out = []
        out.append("Dumping serial stats: %s" % (self.stats(),))
        sdata = self.ffi_main.new('struct pull_queue_message[1024]')
        rdata = self.ffi_main.new('struct pull_queue_message[1024]')
        scount = self.ffi_lib.serialqueue_extract_old(self.serialqueue, 1,
                                                      sdata, len(sdata))
        rcount = self.ffi_lib.serialqueue_extract_old(self.serialqueue, 0,
                                                      rdata, len(rdata))
        out.append("Dumping send queue %d messages" % (scount,))
        for i in range(scount):
            msg = sdata[i]
            cmds = self.msgparser.dump(msg.msg[0:msg.len])
            out.append("Sent %d %f %f %d: %s" % (i, msg.receive_time, msg.sent_time, msg.len, ', '.join(cmds)))
        out.append("Dumping receive queue %d messages" % (rcount,))
        
        for i in range(rcount):
            msg = rdata[i]
            cmds = self.msgparser.dump(msg.msg[0:msg.len])
            out.append("Receive: %d %f %f %d: %s" % (
                i, msg.receive_time, msg.sent_time, msg.len, ', '.join(cmds)))
        return '\n'.join(out)
    
    # Default message handlers
    def _handle_unknown_init(self, params):
        logging.debug("%sUnknown message %d (len %d) while identifying",
                      self.warn_prefix, params['#msgid'], len(params['#msg']))
    
    def handle_unknown(self, params):
        logging.warning("%sUnknown message type %d: %s",
                     self.warn_prefix, params['#msgid'], repr(params['#msg']))
    
    def handle_output(self, params):
        logging.info("%s%s: %s", self.warn_prefix,
                     params['#name'], params['#msg'])

# Class to send a query command and return the received response
class SerialRetryCommand:
    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.last_params = None
        self.serial.register_response(self.handle_callback, name, oid)
    
    def handle_callback(self, params):
        self.last_params = params
    
    def get_response(self, cmds, cmd_queue, minclock=0, reqclock=0):
        retries = 5
        retry_delay = .010
        while retries > 0:
            for cmd in cmds[:-1]:
                self.serial.raw_send(cmd, minclock, reqclock, cmd_queue)
            self.serial.raw_send_wait_ack(cmds[-1], minclock, reqclock, cmd_queue)
            params = self.last_params
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            retries -= 1
            time.sleep(retry_delay)
            retry_delay *= 2.
        self.serial.register_response(None, self.name, self.oid)
        raise error("Unable to obtain '%s' response" % (self.name,))
    


# Attempt to place an AVR stk500v2 style programmer into normal mode
def stk500v2_leave(ser):
    logging.debug("Starting stk500v2 leave programmer sequence")
    util.clear_hupcl(ser.fileno())
    origbaud = ser.baudrate
    # Request a dummy speed first as this seems to help reset the port
    ser.baudrate = 2400
    ser.read(1)
    # Send stk500v2 leave programmer sequence
    ser.baudrate = 115200
    time.sleep(0.100)
    ser.read(4096)
    ser.write(b'\x1b\x01\x00\x01\x0e\x11\x04')
    time.sleep(0.050)
    res = ser.read(4096)
    logging.debug("Got %s from stk500v2", repr(res))
    ser.baudrate = origbaud


# Attempt an arduino style reset on a serial port
def arduino_reset(serialport):
    # First try opening the port at a different baud
    ser = serial.Serial(serialport, 2400, timeout=0, exclusive=True)
    ser.read(1)
    time.sleep(0.100)
    # Then toggle DTR
    ser.dtr = True
    time.sleep(0.100)
    ser.dtr = False
    time.sleep(0.100)
    ser.close()
