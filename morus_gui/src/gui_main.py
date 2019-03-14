from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys

from morus_gui import MorusGUI

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MorusGUI()
    sys.exit(app.exec_())