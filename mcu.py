import serial
import threading

class MCU:
    def __init__(self, port, baudrate):
        
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True
        self.callback_handlers = {}
        self.listener_thread = threading.Thread(target=self.listen_for_callbacks)
        self.listener_thread.start()
    
    # Send a command to the MCU without waiting for a response
    def send_command(self, command):
        self.ser.write((command + '\n').encode(('utf-8')))
        self.ser.flush()
        
    # Send it with response        
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
