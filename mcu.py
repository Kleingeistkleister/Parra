import serial
import threading


class MCU:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True
        self.callback_handlers = {}
        self.listener_thread = threading.Thread(target=self.listen_for_callbacks)
        self.listener_thread.start()

        # Initialize the CommandWrapper
        self.command_wrapper = CommandWrapper(self, "command_format")

    def send_command(self, command):
        self.ser.write((command + '\n').encode(('utf-8')))
        self.ser.flush()

    def send_command_response(self, command):
        self.ser.write((command + '\n').encode(('utf-8')))
        self.ser.flush()
        response = self.ser.readline().decode(('utf-8')).strip()
        return response

    def listen_for_callbacks(self):
        while self.running:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode(('utf-8')).strip()
                self.process_callback(response)

    def process_callback(self, response):
        data = dict(item.split('=') for item in response.split(' '))
        callback_type = data.get('type')
        if callback_type in self.callback_handlers:
            self.callback_handlers[callback_type](data)
        else:
            print(f"Unhandled callback: {response}")

    def register_callback(self, callback_type, handler):
        self.callback_handlers[callback_type] = handler

    def stop(self):
        self.running = False
        self.listener_thread.join()
        self.ser.close()

class CommandWrapper:
    def __init__(self, serial, msgformat, cmd_queue=None):
        self._serial = serial
        msgparser = serial.get_msgparser()
        self._cmd = msgparser.lookup_command(msgformat)
        if cmd_queue is None:
            cmd_queue = serial.get_default_command_queue()
        self._cmd_queue = cmd_queue
        self._msgtag = msgparser.lookup_msgtag(msgformat) & 0xffffffff

    def send(self, data=(), minclock=0, reqclock=0):
        cmd = self._cmd.encode(data)
        self._serial.raw_send(cmd, minclock, reqclock, self._cmd_queue)

    def send_wait_ack(self, data=(), minclock=0, reqclock=0):
        cmd = self._cmd.encode(data)
        self._serial.raw_send_wait_ack(cmd, minclock, reqclock, self._cmd_queue)

    def get_command_tag(self):
        return self._msgtag
