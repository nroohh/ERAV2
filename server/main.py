from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QSizePolicy,
    QSlider, QTableWidget, QHeaderView, QGridLayout, QFrame, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer
import sys
from utils import *
from blue import *
import asyncio
import qasync
from qasync import QEventLoop, asyncSlot
import json
from functools import partial
import time


COMMAND_CHARACTERISTIC_UUID = "b7b88cad-dff1-492c-ab98-2aab1c81da84"; # arm, launch, kill; used to send commands
TARGET_CHARACTERISTIC_UUID = "4fe486b9-93fc-4333-a5af-b8dcea135cf3"; # rx, ry, rz, al; used to control the target values
SETTINGS_CHARACTERISTIC_UUID = "1f8c64d3-1ca4-43c3-88f9-b4c8df79be5a"; # rx: {}, ry: {}, rz: {}, aa: {}, tt: {}; used to control PID gains

blue = Blue()

idv_variables = { # individual variables
    "SERVO_RANGE": {
        "value": 50,
        "description": "limiting threshold value the maximum servo range in degrees"
    },
    "Z2XY_WEIGHT": {
        "value": 0,
        "description": "ratio of pid for roll(z) to pitch(x) and yaw(y) control"
    },
    "SERVO_OFFSETS_0": {
        "value": -17.5,
        "description": "initial offset of servo 0"
    },
    "SERVO_OFFSETS_1": {
        "value": -12.5,
        "description": "initial offset of servo 1"
    },
    "SERVO_OFFSETS_2": {
        "value": -22.5,
        "description": "initial offset of servo 2"
    },
    "SERVO_OFFSETS_3": {
        "value": -9.5,
        "description": "initial offset of servo 3"
    },
}

pid_variables = { # pid variables
    "RX_PG": 0, "RX_IG": 0, "RX_DG": 0,
    "RY_PG": 0, "RY_IG": 0, "RY_DG": 0,
    "RZ_PG": 0, "RZ_IG": 0, "RZ_DG": 0,
    "AL_PG": 0, "AL_IG": 0, "AL_DG": 0,
    "AA_PG": 0, "AA_IG": 0, "AA_DG": 0,
}

commands = {
    "arm": {
        "description": "prepare the vehicle for flight",
        "function": None
    },
    "launch": {
        "description": "start the main loop",
        "function": None
    },
    "kill": {
        "description": "stop the main loop",
        "function": None
    },
    "set": {
        "description": "set all of the external setting variables",
        "function": None
    },
    "sequence": {
        "description": "run sequence",
        "function": None
    },
    "reset": {
        "description": "run reset",
        "function": None
    }
}

parameters = {
    "RX_TGT": {"value": 0, "min": -50, "max": 50, "func": None, "slider": None, "label": None},
    "RY_TGT": {"value": 0, "min": -50, "max": 50, "func": None, "slider": None, "label": None},
    "RZ_TGT": {"value": 0, "min": -50, "max": 50, "func": None, "slider": None, "label": None},
    "AL_TGT": {"value": 0, "min": 0, "max": 100, "func": None, "slider": None, "label": None},
}

class ConfigPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setObjectName("panel")
        self.load()

    def load(self):
        main_layout = QVBoxLayout()

        scroll_content = QWidget()
        content_layout = QVBoxLayout(scroll_content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(5)

        idv_layout = QGridLayout()
        idv_layout.setSpacing(0)
        idv_layout.setContentsMargins(0, 0, 0, 0)

        n = 0
        for name, info in idv_variables.items():
            idv_layout.addWidget(QLabel(f"{name}: "), n, 0)
            idv_layout.addWidget(
                VariableInputBox(idv_variables, name, info["value"], info["description"]), 
                n, 1
            )
            n += 1

        content_layout.addLayout(idv_layout)

        # PID table
        columns = ["PG", "IG", "DG"]
        rows = ["RX", "RY", "RZ", "AA", "AL"]

        table = QTableWidget(len(rows), len(columns))
        table.setHorizontalHeaderLabels(columns)
        table.setVerticalHeaderLabels(rows)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setMinimumHeight(200)

        for r, row in enumerate(rows):
            for c, col in enumerate(columns):
                name = f"{row}_{col}"
                value = pid_variables[name]
                cell_edit = VariableInputCell(pid_variables, name, value)
                table.setCellWidget(r, c, cell_edit.input)

        content_layout.addWidget(table)

        scroll_area = QScrollArea()
        scroll_area.setWidget(scroll_content)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        main_layout.addWidget(scroll_area)
        self.setLayout(main_layout)

class CommandPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setObjectName("panel")

        # link commands
        commands["arm"]["function"] = self.arm
        commands["launch"]["function"] = self.launch
        commands["kill"]["function"] = self.kill
        commands["set"]["function"] = self.set
        commands["sequence"]["function"] = self.sequence
        commands["reset"]["function"] = self.reset

        self.load()

    def load(self):
        main_layout = QVBoxLayout()

        scroll_content = QWidget()
        content_layout = QVBoxLayout(scroll_content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(5)

        for command_name, info in commands.items():
            content_layout.addWidget(
                CommandButton(command_name, info["description"], info["function"])
            )

        content_layout.addStretch()

        scroll_area = QScrollArea()
        scroll_area.setWidget(scroll_content)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        main_layout.addWidget(scroll_area)
        self.setLayout(main_layout)

    @asyncSlot()
    async def arm(self):
        await blue.write(COMMAND_CHARACTERISTIC_UUID, "arm")

    @asyncSlot()
    async def launch(self):
        await blue.write(COMMAND_CHARACTERISTIC_UUID, "launch")

    @asyncSlot()
    async def kill(self):
        await blue.write(COMMAND_CHARACTERISTIC_UUID, "kill")

    @asyncSlot()
    async def reset(self):
        await blue.write(COMMAND_CHARACTERISTIC_UUID, "reset")

    @asyncSlot()
    async def sequence(self):
        for i in range(0, 65, 5):
            idv_variables["SERVO_OFFSETS_1"]["value"] = -i
            idv_variables["SERVO_OFFSETS_3"]["value"] = i
            time.sleep(5)
            await self.set()

    @asyncSlot()
    async def set(self):
        data = {}
        for name, info in idv_variables.items():
            data[name] = info["value"]

        for name, value in pid_variables.items():
            data[name] = value

        json_string = json.dumps(data)
        await blue.write(SETTINGS_CHARACTERISTIC_UUID, json_string)

class BluetoothPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setObjectName("panel")
        self.connected = False
        self.load()

    def load(self):
        main_layout = QVBoxLayout()

        self.scanButton = QPushButton("scan")
        self.scanButton.clicked.connect(self.onClickScan)

        self.scroll_content = QWidget()
        self.BLElist = QVBoxLayout(self.scroll_content)
        self.BLElist.setContentsMargins(0, 0, 0, 0)
        self.BLElist.setSpacing(5)

        scroll_area = QScrollArea()
        scroll_area.setWidget(self.scroll_content)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        scroll_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.scroll_content.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        main_layout.addWidget(scroll_area)
        main_layout.addWidget(self.scanButton)

        self.setLayout(main_layout)

    @asyncSlot()
    async def onClickScan(self):
        self.scanButton.setText("scanning...")
        self.scanButton.setEnabled(False)

        try:
            devices = await blue.scan(5)

            while self.BLElist.count():
                item = self.BLElist.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()

            for device in devices:
                label = BLEServerItem(device.name or "Unknown", device.address)
                label.button.setText("connect")
                label.button.clicked.connect(partial(self.onClick, label, device.address))
                self.BLElist.addWidget(label)

        finally:
            self.scanButton.setText("scan")
            self.scanButton.setEnabled(True)


    @asyncSlot()
    async def onClick(self, widget, address):
        if not self.connected:
            ok = await blue.connect(address)
            if ok:
                self.connected = True
                widget.button.setText("disconnect")
        else:
            ok = await blue.disconnect()
            if ok:
                self.connected = False
                widget.button.setText("connect")
class ControlPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setObjectName("panel")

        # --- Throttling Setup ---
        self.send_timer = QTimer(self)
        # Set the delay (e.g., 100 milliseconds)
        self.send_timer.setInterval(100) 
        # Only run once after being started/restarted
        self.send_timer.setSingleShot(True)
        # Connect the timer's timeout to the Bluetooth send function
        self.send_timer.timeout.connect(self.onUpdate) 
        # ------------------------

        # Update the parameter function mappings to use the new onSliderChange
        parameters["RX_TGT"]["func"] = self.onSliderChangeRX
        parameters["RY_TGT"]["func"] = self.onSliderChangeRY
        parameters["RZ_TGT"]["func"] = self.onSliderChangeRZ
        parameters["AL_TGT"]["func"] = self.onSliderChangeAL

        self.load()

    def load(self):
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        for name, parameter in parameters.items():
            layout = QGridLayout()

            label = QLabel(name)

            parameter["slider"] = QSlider(Qt.Horizontal, self)
            parameter["slider"].setRange(parameter["min"], parameter["max"])
            # Adjust for AL_TGT scaling on initial load
            initial_value = parameter["value"]
            if name == "AL_TGT":
                initial_value = int(parameter["value"] * 10)
            parameter["slider"].setValue(initial_value)

            # Adjust label display for AL_TGT on initial load
            display_value = parameter["value"] if name != "AL_TGT" else parameter["value"] / 10
            parameter["label"] = QLabel(str(display_value))
            parameter["label"].setAlignment(Qt.AlignCenter)

            button = QPushButton("reset")
            # When reset is clicked, set value to 0, which triggers valueChanged, which starts the timer.
            button.clicked.connect(lambda _, s=parameter["slider"]: s.setValue(0))

            # Connect valueChanged to the throttling function
            parameter["slider"].valueChanged.connect(parameter["func"])

            layout.addWidget(label, 0, 0)
            layout.addWidget(parameter["slider"], 1, 0)
            layout.addWidget(parameter["label"], 0, 1)
            layout.addWidget(button, 1, 1)

            main_layout.addLayout(layout)

    # All these functions now only update the value/label and start the timer.
    def onSliderChangeRX(self, value):
        parameters["RX_TGT"]["value"] = value
        parameters["RX_TGT"]["label"].setText(str(value))
        self.send_timer.start() # Start or restart the timer

    def onSliderChangeRY(self, value):
        parameters["RY_TGT"]["value"] = value
        parameters["RY_TGT"]["label"].setText(str(value))
        self.send_timer.start() # Start or restart the timer

    def onSliderChangeRZ(self, value):
        parameters["RZ_TGT"]["value"] = value
        parameters["RZ_TGT"]["label"].setText(str(value))
        self.send_timer.start() # Start or restart the timer

    def onSliderChangeAL(self, value):
        # Update internal value and display label with scaling
        scaled_value = value / 10
        parameters["AL_TGT"]["value"] = scaled_value
        parameters["AL_TGT"]["label"].setText(str(scaled_value))
        self.send_timer.start() # Start or restart the timer

    # This function is now only called by the QTimer's timeout signal.
    @asyncSlot()
    async def onUpdate(self):
        # Stop the timer immediately to ensure it doesn't fire again
        self.send_timer.stop() 
        
        data = {}
        for name, parameter in parameters.items():
            # Use the most recent value set by the onSliderChange functions
            data[name] = parameter["value"] 

        json_string = json.dumps(data)
        # Send the Bluetooth command
        await blue.write(TARGET_CHARACTERISTIC_UUID, json_string)

class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("controls panel")
        self.load()


    def load(self):
        layout = QGridLayout()

        config_panel = ConfigPanel()
        command_panel = CommandPanel()
        bluetooth_panel = BluetoothPanel()
        control_panel = ControlPanel()

        layout.addWidget(config_panel, 0, 0)
        layout.addWidget(command_panel, 1, 0)
        layout.addWidget(bluetooth_panel, 0, 1)
        layout.addWidget(control_panel, 1, 1)

        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        self.setLayout(layout)

        self.setStyleSheet("""
            #panel {
                background-color: #f0f0f0;
                border: 1px solid #cfcfcf;
                border-radius: 8px;
                padding: 10px;
            }
        """)

async def main(app):
    close_event = asyncio.Event()
    app.aboutToQuit.connect(close_event.set)
    window = Window()
    window.show()
    await close_event.wait()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    qasync.run(main(app))