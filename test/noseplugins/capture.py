import nose
import nose.plugins.capture
import sys
class Capture(nose.plugins.capture.Capture):
    def addSuccess(self,test):
        test.capturedOutput = self.buffer
    def addCaptureToErr(self, ev, output):
        return ev
#         return '\n'.join([str(ev) , ln('>> begin captured stdout <<'),
#                           output, ln('>> end captured stdout <<')])
