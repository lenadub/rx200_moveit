import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RX200 Joint Control")
        self.resize(860, 520)

        # Central widget and window
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        #PLACEHOLDER LABALES
        ph = QLabel("PLACEHOLDER")
        ph.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(ph)
        
def main():
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())
        
if __name__ == "__main__":
    main()
