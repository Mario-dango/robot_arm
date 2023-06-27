import serial.tools.list_ports


class Model:
    def __init__(self):
        self.ports = []

    def get_available_ports(self):
        ports = serial.tools.list_ports.comports()
        self.ports = [port.device for port in ports]
        return self.ports

    def send_data(self, port, data):
        # Aquí puedes agregar la lógica para enviar los datos al puerto COM seleccionado
        pass

    def receive_data(self, port):
        # Aquí puedes agregar la lógica para recibir datos del puerto COM y
        # almacenarlos en una variable 'received_data'.
        received_data = "Datos recibidos desde el COM"
        return received_data
