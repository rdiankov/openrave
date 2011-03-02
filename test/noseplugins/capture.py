"""Derives from the nose.plugins.capture.Capture class and provides an addSuccess callback. It also removes the system out put from the error log, this is passed through the capturedOutput.
"""
import nose
import nose.plugins.capture
import sys
class Capture(nose.plugins.capture.Capture):
    def addSuccess(self,test):
        test.capturedOutput = self.buffer
    def addError(self,test, err):
        test.capturedOutput = self.buffer
    def addFailure(self,test, err):
        test.capturedOutput = self.buffer
    def addCaptureToErr(self, ev, output):
        return ev
#         return '\n'.join([str(ev) , ln('>> begin captured stdout <<'),
#                           output, ln('>> end captured stdout <<')])
