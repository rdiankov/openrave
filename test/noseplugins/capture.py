"""Derives from the nose.plugins.capture.Capture class and provides an addSuccess callback. It also removes the system out put from the error log, this is passed through the capturedOutput.
"""
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
#     def formatError(self, test, err):
#         """Add captured output to failure report.
#         """
#         sys.stderr.write('formatFailure: %s\n'%self.buffer)
#         return nose.plugins.capture.Capture.formatError(self,test, err)
#     def afterTest(self, test):
#         pass
#         return '\n'.join([str(ev) , ln('>> begin captured stdout <<'),
#                           output, ln('>> end captured stdout <<')])
