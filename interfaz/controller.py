from model import Model
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMessageBox
from view import View, ViewUpdater


class Controller:
    def __init__(self):
        self.model = Model()
        self.view = View()
        self.view_updater = ViewUpdater(self.view)
        self.setup_ui()
        self.setup_signals()
        

        self.receive_timer = QTimer()  # Temporizador para la recepción continua de datos
        self.receive_timer.timeout.connect(self.receive_data)
        self.serial_port = None

    def setup_ui(self):
        self.view.show()
        self.update_available_ports()

    def setup_signals(self):
        self.view.button_send.clicked.connect(self.send_data)

    def update_available_ports(self):
        ports = self.model.get_available_ports()
        self.view_updater.update_ports(ports)

    def send_data(self):
        selected_port = self.view_updater.get_selected_port()
        data = self.view_updater.get_input_data()

        if not selected_port:
            QMessageBox.warning(self.view, "Advertencia", "Por favor, selecciona un puerto COM.")
            return

        if not self.serial_port or selected_port != self.serial_port.port:
            # Si el puerto no está abierto o se seleccionó un puerto diferente, abrir el nuevo puerto
            if self.serial_port:
                self.serial_port.close()

            self.serial_port = self.model.open_serial_port(selected_port)
            self.receive_timer.start(100)  # Inicia el temporizador para la recepción continua de datos

        self.model.send_data(selected_port, data)
        self.view_updater.append_output_text(f"TX: {data}")
        self.view_updater.clear_input_data()

    def receive_data(self):
        received_data = self.model.receive_data()
        if received_data:
            self.view_updater.append_output_text(f"RX: {received_data}")

    def cleanup(self):
        if self.serial_port:
            self.serial_port.close()
        self.receive_timer.stop()

    # Resto del código