#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 daniel (daniel.kappler@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Control and run the openrave python examples with a pyqt gui.

.. examplepre-block:: qtexampleselector

Description
-----------

This example shows how to run a qt-gui in python and control an openrave python script including viewer and terminal input.
Therefore the openravepy.examples dictionary is used to provide all available openravepy examples to the qt-gui.
To abort an example just restart the OpenRaveServer.
If you want to start an example with some arguments
just insert them in the "Arguments for Example" box with space seperation.

.. image:: ../../images/examples/qtexampleselector.jpg
  :width: 640

.. examplepost-block:: qtexampleselector
"""


from __future__ import with_statement # for python 2.5
__author__ = 'Daniel Kappler'
__copyright__ = '2011 Daniel Kappler (daniel.kappler@gmail.com)'

import sys, os, re, logging, signal, traceback
from multiprocessing import Process,Pipe
from threading import Thread

from PyQt4 import QtGui, QtCore

logger = logging.getLogger('PyqtControl')

#Change to main Gui Server which just inits a Qt or OpenRAVE Gui
class Example(Thread):
    def __init__(self,mod,args):
        super(Example,self).__init__()
        self.mod = mod
        self.args = args

    def run(self):
        try:
            sys.stdin = open("/dev/tty")
            getattr(self.mod, "run")(self.args)

        except:
            logger.info("::TRACEBACK:: of example gui functional.")
            traceback.print_exc(file=sys.stdout)


class CallbackHandler(QtCore.QThread):
    def __init__(self,pipe,callback=None):
        super(CallbackHandler,self).__init__()
        self.callback = callback
        self.pipe = pipe

    def run(self):
        resultValue = self.pipe.recv()
        msg = [self.callback,resultValue]
        self.emit(QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),msg)

class OpenRaveServer(object):
    '''
    Control server to run the benchmark in its own process.
    '''

    def __init__(self,pipe):
        '''
        Setup the shared memory data structure model and initialize the control parts.
        '''
        self.pipe = pipe
        self.running = True
        self._run()

    def _run(self):
        '''
        Main server loop which waits for input from the qt-gui.
        '''
        while(self.running):
            (functionName,args) = self.pipe.recv()
            rValue = self.executeFunction(functionName, args)
            if (self.running):
                self.pipe.send(rValue)

    def executeFunction(self,name,args):
        try:
            if name == "___LOAD_EXAMPLE___":
                return self.LoadExample(args[0],args[1])
            if name == "___CLOSE___":
                try:
                    self.example.terminate()
                except:
                    pass
                self.running=False
                return True
        except Exception as e:
            logger.error(str(e))

        logger.info("No such function "+str(name)+" available.")
        return False

    def LoadExample(self,fileName,args):
        try:
            import openravepy.examples
            mod = getattr(openravepy.examples, fileName)
            self.example = Example(mod,args)
            self.example.start()
            return True
        except Exception as e:
            logger.debug(str(e))
        return False

class Server(object):
    '''
    Control server to run the benchmark in its own process.
    '''

    def __init__(self):
        '''
        Setup the shared memory data structure model and initialize the control parts.
        '''
        self.openRaveServers = []
        self.running = True
        self.orgui = None
        self.qtgui = None
        (self.pipeQtControl, self.pipeORControl) = Pipe()
        (self.pipeQtServer, self.pipeServer) = Pipe()
        self.StartQtGuiControl()
        self._run()

    def _run(self):
        '''
        Main server loop which waits for input from the qt-gui.
        '''
        while(self.running):
            (functionName,args,handleCallback) = self.pipeServer.recv()
            rValue = self.executeFunction(functionName, args)
            if handleCallback:
                self.pipeServer.send(rValue)

    def executeFunction(self,name,args):
        logger.debug("server execute function "+str(name))
        if name == "___START_OPENRAVE_SERVER___":
            return self.StartOpenRaveGuiServer()
        if name == "___CLOSE___":
            self.running = False
            try:
                self.qtgui.terminate()
                self.orgui.terminate()
            except:
                pass
            return True
        return False

    def StartOpenRaveGuiServer(self):
        if self.orgui:
            self.orgui.terminate()
        self.orgui = Process(target=OpenRaveServer,args=(self.pipeORControl,))
        self.orgui.start()
        return True

    def StartQtGuiControl(self):
        self.qtgui = Process(target=self._StartQtGuiControl)
        self.qtgui.start()
        return True

    def _StartQtGuiControl(self):
        app = QtGui.QApplication(sys.argv)
        form = MainWindow(self.pipeQtControl,self.pipeQtServer)
        form.show()
        app.exec_()

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(352, 283)
        MainWindow.setMinimumSize(QtCore.QSize(352, 185))
        MainWindow.setMaximumSize(QtCore.QSize(352, 250))
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.teArgs = QtGui.QTextEdit(self.centralwidget)
        self.teArgs.setGeometry(QtCore.QRect(10, 120, 331, 121))
        self.teArgs.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.teArgs.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.teArgs.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.teArgs.setObjectName(_fromUtf8("teArgs"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(160, 0, 171, 16))
        self.label.setObjectName(_fromUtf8("label"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(10, 100, 171, 16))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.layoutWidget = QtGui.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 10, 113, 89))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.pbOR = QtGui.QPushButton(self.layoutWidget)
        self.pbOR.setObjectName(_fromUtf8("pbOR"))
        self.verticalLayout.addWidget(self.pbOR)
        self.pbRun = QtGui.QPushButton(self.layoutWidget)
        self.pbRun.setObjectName(_fromUtf8("pbRun"))
        self.verticalLayout.addWidget(self.pbRun)
        self.pbClose = QtGui.QPushButton(self.layoutWidget)
        self.pbClose.setObjectName(_fromUtf8("pbClose"))
        self.verticalLayout.addWidget(self.pbClose)
        self.cbExamples = QtGui.QComboBox(self.centralwidget)
        self.cbExamples.setGeometry(QtCore.QRect(150, 20, 191, 21))
        self.cbExamples.setObjectName(_fromUtf8("cbExamples"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 352, 20))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        self.teArgs.setHtml(QtGui.QApplication.translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Sans Serif\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"></p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "Available Examples:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("MainWindow", "Arguments for Example:", None, QtGui.QApplication.UnicodeUTF8))
        self.pbOR.setText(QtGui.QApplication.translate("MainWindow", "OpenRaveServer", None, QtGui.QApplication.UnicodeUTF8))
        self.pbRun.setText(QtGui.QApplication.translate("MainWindow", "RunExample", None, QtGui.QApplication.UnicodeUTF8))
        self.pbClose.setText(QtGui.QApplication.translate("MainWindow", "Close", None, QtGui.QApplication.UnicodeUTF8))


class MainWindow(QtGui.QMainWindow,Ui_MainWindow):
    def __init__(self,pipeOR,pipeServer):
        super(MainWindow,self).__init__(None)
        self.pipeServer = pipeServer
        self.pipeOR = pipeOR

        self.CallbackHandler = CallbackHandler(self.pipeServer)
        self.connect(self.CallbackHandler,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.__index = 0
        self.setupUi(self)

        import openravepy.examples
        fileNames = dir(openravepy.examples)
        fileNames.sort(key=str.lower)
        for filename in fileNames:
            if re.match("__\w*__\Z",filename):
                continue
            self.cbExamples.addItem(QtCore.QString(filename))

        self.buttons = []
        self.buttons.append(self.pbOR)
        self.buttons.append(self.pbRun)
        self.pbRun.setEnabled(False)

    def ButtonsLock(self):
        self.locks = []
        for button in self.buttons:
            self.locks.append(button.isEnabled())
            button.setEnabled(False)

    def ButtonsUnlock(self):
        for i,lock in enumerate(self.locks):
            self.buttons[i].setEnabled(lock)

    def close(self):
        self.SendToOR("___CLOSE___")
        self.SendToServer("___CLOSE___")
        QtGui.QApplication.quit()

    def closeEvent(self, event):
        self.close()

    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.close()

    @QtCore.pyqtSignature("")
    def on_pbOR_clicked(self):
        self.pbRun.setEnabled(True)
        self.ButtonsLock()
        self.orRunning = False
        self.SendToServer("___START_OPENRAVE_SERVER___",callback=self.CallbackOR)

    def CallbackOR(self,args):
        self.ButtonsUnlock()

    @QtCore.pyqtSignature("")
    def on_pbRun_clicked(self):
        if self.orRunning:
            self.on_pbOR_clicked()
        self.orRunning = True
        args = str(self.teArgs.toPlainText())
        if not args:
            args = []
        else:
            args = args.split(" ")
        self.SendToOR("___LOAD_EXAMPLE___",[str(self.cbExamples.currentText()),args])

    def SendToServer(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandler.callback = callback
            self.CallbackHandler.start()
            handleCallback=True
        self.pipeServer.send([command,args,handleCallback])

    def SendToOR(self,command,args=None):
        self.pipeOR.send([command,args])

    def HandleCallback(self,msg):
        if(len(msg) == 2):
            if(msg[0] is not None):
                try:
                    msg[0](msg[1])
                except Exception as e:
                    logger.error(str(e))
            return
        logger.error("ERROR in request format")

def main(env,options):
    "Main example code."
    global logger
    lhandler =logging.StreamHandler(sys.stdout)
    lhandler.setFormatter(logging.Formatter("%(levelname)-10s:: %(filename)-20s - %(lineno)4d :: %(message)s"))
    logger.setLevel(logging.INFO)
    logger.addHandler(lhandler)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    server = Server()

from optparse import OptionParser

def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Control and run the openrave python examples with a pyqt gui.', usage='openrave.py --example qtexampleselector [options]')
    (options, leftargs) = parser.parse_args(args=args)
    main(None,options)


if __name__ == "__main__":
    run()


