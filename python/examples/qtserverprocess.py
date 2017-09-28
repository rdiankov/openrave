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
"""Pyqt example to demonstrate how openrave elements can be controlled by a qt-gui.

.. examplepre-block:: qtserverprocess

Description
-----------

This example shows how a qt-gui can control openrave functionality.
If an environment with a robot is loaded,
a random pose is generated for which a trajectory is planned.
This trajectory can be replayed using the qt-gui.


.. image:: ../../images/examples/qtexampleselector.jpg
  :width: 400


.. examplepost-block:: qtserverprocess
"""

from __future__ import with_statement # for python 2.5
__author__ = 'Daniel Kappler'
__copyright__ = '2011 Daniel Kappler (daniel.kappler@gmail.com)'

import sys, os, re, logging, signal
from numpy import random
from multiprocessing import Process,Pipe
from threading import Thread
from openravepy import *

from PyQt4 import QtGui, QtCore

logger = None

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
    classdocs
    '''

    def __init__(self,pipe):
        '''
        Constructor
        '''
        self.pipe = pipe
        self.running = True
        self.orenv = Environment()
        self._run()


    def __del__(self):
        RaveDestroy()

    def _run(self):
        while(self.running):
            (functionName,args,handleCallback) = self.pipe.recv()
            rValue,rMessage = self.executeFunction(functionName, args)
            if handleCallback:
                self.pipe.send([rValue,rMessage])

    def executeFunction(self,name,args):
        rValue = None
        rMessage = "Function with "+name+" not available"
        if name in dir(self):
            if(args is None):
                rValue,rMessage = getattr(self,name)()
            else:
                rValue,rMessage = getattr(self,name)(args)
        return rValue,rMessage

    def StartGui(self):
        try:
            self.orenv.SetViewer('qtcoin')
            return True,None
        except:
            pass
        return None,"Please start OpenRAVE first."

    def LoadEnv(self,env):
        try:
            self.orenv.Reset()
            if((env is not None) and self.orenv.Load(env) == False):
                return None, "Could not load "+env+"."

            self.robot = self.orenv.GetRobots()[0]
            self.manip = self.robot.GetActiveManipulator()
            return True,None
        except:
            pass
        return None, "Please start OpenRAVE first."

    def UpdateTrajectory(self,pos):
        with self.orenv:
            self.robot.GetController().SetDesired(self.trajdata[pos])
        return "Trajectory position "+str(pos),[pos,len(self.trajdata)-1]

    def waitrobot(self):
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)

    def CalcTrajectory(self):
        notFound = True
        with self.orenv:
            while notFound:
                logger.debug("search for goal")
                notFound = False
                try:
                    Tnewgoals = []
                    Tnewgoals.append(self.manip.GetEndEffectorTransform() +(random.rand(4,4)-0.5))
                    data = interfaces.BaseManipulation(self.robot).MoveToHandPosition(matrices=Tnewgoals,maxiter=1000,maxtries=1,seedik=4,execute=True,outputtraj=True).split(" ")
                except Exception as e:
                    notFound = True
                    logger.debug(str(e))

        skip = False
        jointValues = []
        self.trajdata = []
        size = self.robot.GetDOF()
        valueIter = iter(data)

        # remove trailing trajectory information
        next(valueIter)
        next(valueIter)
        next(valueIter)
        next(valueIter)
        try:
            while True:
                jointValues = []
                # time value
                value = next(valueIter)

                # only the arm joints
                for i in range(size):
                    value = next(valueIter)
                    jointValues.append(float(value))
                self.trajdata.append(jointValues)

                # go on till the next line
                while (len(next(valueIter)) is not 0):
                    pass

                value = next(valueIter)
        except StopIteration:
            pass

        return True, [len(self.trajdata)-1,len(self.trajdata)-1]

    def quit(self):
        RaveDestroy()
        self.running = False
        return True,None

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
        MainWindow.resize(352, 389)
        MainWindow.setMinimumSize(QtCore.QSize(352, 389))
        MainWindow.setMaximumSize(QtCore.QSize(352, 389))
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.teEnv = QtGui.QTextEdit(self.centralwidget)
        self.teEnv.setGeometry(QtCore.QRect(160, 220, 149, 25))
        self.teEnv.setMaximumSize(QtCore.QSize(16777215, 25))
        self.teEnv.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.teEnv.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.teEnv.setObjectName(_fromUtf8("teEnv"))
        self.teOutput = QtGui.QTextEdit(self.centralwidget)
        self.teOutput.setGeometry(QtCore.QRect(160, 40, 181, 139))
        self.teOutput.setReadOnly(True)
        self.teOutput.setTextInteractionFlags(QtCore.Qt.NoTextInteraction)
        self.teOutput.setObjectName(_fromUtf8("teOutput"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(160, 20, 171, 16))
        self.label.setObjectName(_fromUtf8("label"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(160, 200, 171, 16))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.layoutWidget = QtGui.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 280, 331, 68))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_3.setMargin(0)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.pbTrajectory = QtGui.QPushButton(self.layoutWidget)
        self.pbTrajectory.setObjectName(_fromUtf8("pbTrajectory"))
        self.horizontalLayout.addWidget(self.pbTrajectory)
        self.pbBackward = QtGui.QPushButton(self.layoutWidget)
        self.pbBackward.setObjectName(_fromUtf8("pbBackward"))
        self.horizontalLayout.addWidget(self.pbBackward)
        self.pbForward = QtGui.QPushButton(self.layoutWidget)
        self.pbForward.setObjectName(_fromUtf8("pbForward"))
        self.horizontalLayout.addWidget(self.pbForward)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.hsTimeLine = QtGui.QSlider(self.layoutWidget)
        self.hsTimeLine.setOrientation(QtCore.Qt.Horizontal)
        self.hsTimeLine.setObjectName(_fromUtf8("hsTimeLine"))
        self.verticalLayout_3.addWidget(self.hsTimeLine)
        self.layoutWidget1 = QtGui.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(10, 40, 113, 128))
        self.layoutWidget1.setObjectName(_fromUtf8("layoutWidget1"))
        self.verticalLayout = QtGui.QVBoxLayout(self.layoutWidget1)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.pbStartOpenRAVE = QtGui.QPushButton(self.layoutWidget1)
        self.pbStartOpenRAVE.setObjectName(_fromUtf8("pbStartOpenRAVE"))
        self.verticalLayout.addWidget(self.pbStartOpenRAVE)
        self.pbStartGui = QtGui.QPushButton(self.layoutWidget1)
        self.pbStartGui.setObjectName(_fromUtf8("pbStartGui"))
        self.verticalLayout.addWidget(self.pbStartGui)
        self.pbLoadEnv = QtGui.QPushButton(self.layoutWidget1)
        self.pbLoadEnv.setObjectName(_fromUtf8("pbLoadEnv"))
        self.verticalLayout.addWidget(self.pbLoadEnv)
        self.pbClose = QtGui.QPushButton(self.layoutWidget1)
        self.pbClose.setObjectName(_fromUtf8("pbClose"))
        self.verticalLayout.addWidget(self.pbClose)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 352, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        self.teEnv.setHtml(QtGui.QApplication.translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Myriad Web\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Sans Serif\';\">data/wamtest1.env.xml</span></p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "OpenRAVE Server Response", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("MainWindow", "Path to Environment file.", None, QtGui.QApplication.UnicodeUTF8))
        self.pbTrajectory.setText(QtGui.QApplication.translate("MainWindow", "Calc Trajectory", None, QtGui.QApplication.UnicodeUTF8))
        self.pbBackward.setText(QtGui.QApplication.translate("MainWindow", "step back", None, QtGui.QApplication.UnicodeUTF8))
        self.pbForward.setText(QtGui.QApplication.translate("MainWindow", "step forward", None, QtGui.QApplication.UnicodeUTF8))
        self.pbStartOpenRAVE.setText(QtGui.QApplication.translate("MainWindow", "start OpenRAVE", None, QtGui.QApplication.UnicodeUTF8))
        self.pbStartGui.setText(QtGui.QApplication.translate("MainWindow", "start GUI", None, QtGui.QApplication.UnicodeUTF8))
        self.pbLoadEnv.setText(QtGui.QApplication.translate("MainWindow", "load Env", None, QtGui.QApplication.UnicodeUTF8))
        self.pbClose.setText(QtGui.QApplication.translate("MainWindow", "Close", None, QtGui.QApplication.UnicodeUTF8))


class MainWindow(QtGui.QMainWindow,Ui_MainWindow):

    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.closeAll()


    def __init__(self,pipeOR,pipeServer):
        super(MainWindow,self).__init__(None)
        self.pipeServer = pipeServer
        self.pipeOR = pipeOR


        self.CallbackHandler = CallbackHandler(self.pipeOR)
        self.connect(self.CallbackHandler,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.__index = 0
        self.setupUi(self)

        self.buttons = []
        self.buttons.append(self.pbTrajectory)
        self.buttons.append(self.pbForward)
        self.buttons.append(self.pbBackward)
        self.buttons.append(self.hsTimeLine)
        self.pbTrajectory.setEnabled(False)
        self.pbForward.setEnabled(False)
        self.pbBackward.setEnabled(False)
        self.hsTimeLineActive = False

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
    def on_pbStartOpenRAVE_clicked(self):
        self.updateTeOutput("Starting OpenRave.")
        self.SendToServer("___START_OPENRAVE_SERVER___")

    @QtCore.pyqtSignature("")
    def on_pbStartGui_clicked(self):
        self.updateTeOutput("Starting qtcoin gui.")
        self.SendToOR("StartGui")

    @QtCore.pyqtSignature("")
    def on_pbLoadEnv_clicked(self):
        self.updateTeOutput("Loading the environment.")
        self.hsTimeLine.setEnabled(False)
        self.pbForward.setEnabled(False)
        self.pbBackward.setEnabled(False)
        self.SendToOR("LoadEnv", str(self.teEnv.toPlainText()),self.enableTrajectoryPlanning)

    @QtCore.pyqtSignature("")
    def on_pbTrajectory_clicked(self):
        self.ButtonsLock()
        self.updateTeOutput("Calculate new random trajectory.")
        self.hsTimeLineActive = False
        self.SendToOR("CalcTrajectory", callback=self.enableTrajectoryControl)

    @QtCore.pyqtSignature("")
    def on_pbForward_clicked(self):
        self.ButtonsLock()
        self.updateTeOutput("Next step of the rajectory.")
        self.hsTimeLineActive = False
        self.SendToOR("UpdateTrajectory", (self.hsTimeLine.value()+1),self.enableTrajectoryControl)

    @QtCore.pyqtSignature("")
    def on_pbBackward_clicked(self):
        self.ButtonsLock()
        self.updateTeOutput("Previous step of the trajectory.")
        self.hsTimeLineActive = False
        self.SendToOR("UpdateTrajectory", (self.hsTimeLine.value()-1),self.enableTrajectoryControl)

    @QtCore.pyqtSignature("int")
    def on_hsTimeLine_valueChanged(self,value):
        if self.hsTimeLineActive:
            self.SendToOR("UpdateTrajectory", value,self.enableTrajectoryControl)
            self.hsTimeLine.setEnabled(False)

    def enableTrajectoryPlanning(self,data):
        self.pbTrajectory.setEnabled(True)

    def enableTrajectoryControl(self,traj):
        self.ButtonsUnlock()
        self.pbForward.setEnabled(False)
        self.pbBackward.setEnabled(False)
        if len(traj) == 2:
            if traj[0] < traj[1]:
                self.pbForward.setEnabled(True)
            if traj[0] > 0:
                self.pbBackward.setEnabled(True)

            self.hsTimeLine.setMaximum(traj[1])
            self.hsTimeLine.setValue(traj[0])
            self.hsTimeLine.setEnabled(True)
            self.hsTimeLineActive = True
        else:
            self.updateTeOutput("ERROR: result was in the wrong format")

    def updateTeOutput(self,text):
        content = str(text)
        content += "\n"
        content += str(self.teOutput.toPlainText())
        self.teOutput.setText(content)

    def CallbackOR(self,args):
        self.ButtonsUnlock()

    def SendToServer(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandler.callback = callback
            self.CallbackHandler.start()
            handleCallback=True
        self.pipeServer.send([command,args,handleCallback])

    def SendToOR(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandler.callback = callback
            self.CallbackHandler.start()
            handleCallback=True
        self.pipeOR.send([command,args,handleCallback])

    def HandleCallback(self,msg):
        if(len(msg) == 2):
            if(msg[0] is not None):
                self.updateTeOutput(msg[1][0])
                try:
                    msg[0](msg[1][1])
                except Exception as e:
                    logger.error(str(e))
            else:
                self.updateTeOutput("ERROR: "+msg[1][0])
            return
        logger.error("ERROR in request format")

def main(env,options):
    "Main example code."
    global logger
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    logger = logging.getLogger('PyqtControl')
    lhandler =logging.StreamHandler(sys.stdout)
    lhandler.setFormatter(logging.Formatter("%(levelname)-10s:: %(filename)-20s - %(lineno)4d :: %(message)s"))
    logger.setLevel(logging.INFO)
    logger.addHandler(lhandler)
    server = Server()

from optparse import OptionParser

def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Pyqt example to demonstrate how openrave elements can be controlled by a qt-gui.', usage='openrave.py --example qtserverprocess [options]')
    (options, leftargs) = parser.parse_args(args=args)
    main(None,options)

if __name__ == "__main__":
    run()
