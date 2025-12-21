from PyQt5.QtWidgets import QMainWindow, QLabel, QComboBox, QLineEdit, QPushButton, QTextEdit


class View(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Comunicación Serial")
        self.setGeometry(100, 100, 400, 300)

        self.label_select_port = QLabel("Seleccionar puerto COM:", self)
        self.label_select_port.setGeometry(20, 20, 170, 20)

        self.combo_ports = QComboBox(self)
        self.combo_ports.setGeometry(180, 20, 170, 20)

        self.label_send_data = QLabel("Enviar datos al dispositivo:", self)
        self.label_send_data.setGeometry(20, 60, 170, 20)

        self.text_input = QLineEdit(self)
        self.text_input.setGeometry(180, 60, 170, 20)

        self.button_send = QPushButton("Enviar", self)
        self.button_send.setGeometry(100, 100, 80, 25)
        
        self.button_refresh = QPushButton("Refrescar", self)
        self.button_refresh.setGeometry(240, 100, 80, 25)

        self.label_sent_received = QLabel("Datos enviados y recibidos:", self)
        self.label_sent_received.setGeometry(20, 140, 160, 20)

        self.text_output = QTextEdit(self)
        self.text_output.setGeometry(20, 170, 350, 100)
        self.text_output.setReadOnly(True)
                
        self.set_stylesheet()

    def set_stylesheet(self):
        with open("style.css", "r") as file:
            style_sheet = file.read()
            self.setStyleSheet(style_sheet)


class ViewUpdater:
    def __init__(self, view):
        self.view = view

    def update_ports(self, ports):
        self.view.combo_ports.clear()
        self.view.combo_ports.addItems(ports)

    def get_selected_port(self):
        return self.view.combo_ports.currentText()

    def get_input_data(self):
        return self.view.text_input.text()

    def clear_input_data(self):
        self.view.text_input.clear()

    def append_output_text(self, text):
        self.view.text_output.append(text)
