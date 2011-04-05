import docutils.nodes
from docutils.parsers.rst import Directive
import subprocess, shlex

class ShellCommandDirective(Directive):
    required_arguments = 0
    optional_arguments = 0
    final_argument = False
    option_spec = {}
    has_content = True
    def run(self):
        self.assert_has_content()
        args = shlex.split(self.content[0].encode('ascii'))
        p = subprocess.Popen(args,stdout=subprocess.PIPE)
        text = p.communicate()[0]
        if p.returncode != 0:
            raise ValueError('subprocess.Popen failed: %s'%args)
        
        return [docutils.nodes.literal_block(text=text)]

def setup(app):
    app.add_directive('shell-block', ShellCommandDirective)
