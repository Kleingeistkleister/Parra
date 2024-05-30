import serial
import threading

class MCUConnection:
    def __init__(self, port, baudrate=250000):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True
        self.callback_handlers = {}
        self.listener_thread = threading.Thread(target=self.listen_for_callbacks)
        self.listener_thread.start()

    def send_command(self, command):
        self.ser.write((command + '\n').encode())
        self.ser.flush()
        response = self.ser.readline().decode().strip()
        return response

    def listen_for_callbacks(self):
        while self.running:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
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
