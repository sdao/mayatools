SUM_UI_FILE = "sum.ui"

from maya import cmds
from maya import mel
from maya import OpenMayaUI as ui

from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtUiTools import *
from shiboken import wrapInstance

mayaMainWindowPtr = ui.MQtUtil.mainWindow()
mayaMainWindow = wrapInstance(long(mayaMainWindowPtr), QWidget)

loader = QUiLoader()
file = QFile(SUM_UI_FILE)
file.open(QFile.ReadOnly)
myGui = loader.load(file, parentWidget = mayaMainWindow)
file.close()

def doSum():
    num1 = float(myGui.num1.text())
    num2 = float(myGui.num2.text())
    sumVal = num1 + num2
    myGui.sumResult.setText(str(sumVal))

myGui.sumButton.clicked.connect(doSum)
myGui.show()
