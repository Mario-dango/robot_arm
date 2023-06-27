from model import Model
from PyQt5.QtWidgets import QApplication, QMessageBox
from view import View, ViewUpdater


class Controller:
    def __init__(self):
        self.model = Model()
        self.view = View()
        self.view_updater = ViewUpdater(self.view)
        self.setup_ui()
        self.setup_signals()

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

        self.model.send_data(selected_port, data)
        self.view_updater.append_output_text(f"TX: {data}")

        received_data = self.model.receive_data(selected_port)
        self.view_updater.append_output_text(f"RX: {received_data}")

        self.view_updater.clear_input_data()
