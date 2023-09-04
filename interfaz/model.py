import serial.tools.list_ports


class Model:
    def __init__(self):
        self.ports = []
        self.serial_port = None

    def get_available_ports(self):
        ports = serial.tools.list_ports.comports()
        self.ports = [port.device for port in ports]
        return self.ports

    def send_data(self, port, data):
        # Aquí puedes agregar la lógica para enviar los datos al puerto COM seleccionado
        pass
    
    def open_serial_port(self, port):
        self.serial_port = serial.Serial(port, 115200)  # Configura el puerto con la velocidad de transmisión adecuada

    def receive_data(self):
        if self.serial_port:
            received_data = self.serial_port.read_all().decode()
            return received_data

        return ""
