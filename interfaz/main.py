import sys
from PyQt5.QtWidgets import QApplication
from controller import Controller


if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = Controller()
    sys.exit(app.exec_())
