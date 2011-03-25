"""Derives from the nose.plugins.capture.Capture class and provides an addSuccess callback. It also removes the system out put from the error log, this is passed through the capturedOutput.
"""
__author__ = "Rosen Diankov (rosen.diankov@gmail.com)"
import nose
import nose.plugins.capture
import sys
class Capture(nose.plugins.capture.Capture):
    def addSuccess(self,test):
        test.capturedOutput = self.buffer
    def addError(self,test, err):
        if not hasattr(test,'capturedOutput'):
            test.capturedOutput = self.buffer
        sys.stderr.write('error: %s\n'%self.buffer)
    def addFailure(self,test, err):
        if not hasattr(test,'capturedOutput'):
            test.capturedOutput = self.buffer
        sys.stderr.write('failure: %s\n'%self.buffer)
    def addCaptureToErr(self, ev, output):
        return ev
