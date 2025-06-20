#!/bin/bash
pyinstaller --onefile --windowed --add-binary "compiled_out/can_server:." src/pyqt_can_gui.py
