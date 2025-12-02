from PyQt5.QtWidgets import (
    QWidget, QLabel, QPushButton, QHBoxLayout, QLineEdit
)
from PyQt5.QtCore import Qt

class VariableInputBox(QWidget):
    def __init__(self, var_list, var_name, var_value, description, parent=None):
        super().__init__(parent)
        self.var_list = var_list
        self.var_name = var_name
        self.input = QLineEdit(str(var_value))
        self.input.setPlaceholderText(str(description))
        layout = QHBoxLayout()
        layout.addWidget(self.input)
        self.setLayout(layout)

        def onChange(value):
            if value.strip() == "":
                return
            try:
                var_list[self.var_name]["value"] = float(value)
            except ValueError:
                pass
        
        self.input.textChanged.connect(onChange)

class VariableInputCell(QWidget):
    def __init__(self, var_list, var_name, var_value, parent=None):
        super().__init__(parent)
        self.var_list = var_list
        self.var_name = var_name
        self.input = QLineEdit(str(var_value))
        self.input.setAlignment(Qt.AlignCenter)

        def onChange(value):
            if value.strip() == "":
                return
            try:
                var_list[str(self.var_name)] = float(value)
            except ValueError:
                pass
        
        self.input.textChanged.connect(onChange)

class CommandButton(QWidget): 
    def __init__(self, command_name, command_description, command_function, parent=None):
        super().__init__(parent)
        self.command_name = command_name
        self.button = QPushButton(str(self.command_name))
        self.button.setToolTip(command_description)
        layout = QHBoxLayout()
        layout.addWidget(self.button)
        self.setLayout(layout)
        self.button.clicked.connect(command_function)

class BLEServerItem(QWidget):
     def __init__(self, server_name, server_address, parent=None):
        super().__init__(parent)
        self.server_name = server_name
        self.server_address = server_address
        self.addressLabel = QLabel(self.server_address)
        self.nameLabel = QLabel(self.server_name)
        self.button = QPushButton("connect")
        layout = QHBoxLayout()
        layout.addWidget(self.nameLabel)
        layout.addWidget(self.addressLabel)
        layout.addWidget(self.button)
        self.setLayout(layout)
