"""Generates the preamble and the postscript for examples"""
import docutils.nodes
from docutils.parsers.rst import Directive, Parser

class ExamplePreBlockDirective(Directive):
    required_arguments = 1
    optional_arguments = 0
    final_argument = False
    option_spec = {'image-width': int}
    has_content = False
    def run(self):
        rawtext = """
.. image:: ../../images/examples/%s.jpg
  :width: %d

**Running the Example**::

  openrave.py --example %s

"""%(self.arguments[0],self.options.get('image-width',640),self.arguments[0])
        parser = Parser()
        document = docutils.utils.new_document("<partial node>")
        document.settings = self.state.document.settings
        parser.parse(rawtext,document)
        return document.children

class ExamplePostBlockDirective(Directive):
    required_arguments = 1
    optional_arguments = 0
    final_argument = False
    option_spec = {}
    has_content = False
    def run(self):
        rawtext = """
Command-line
------------

.. shell-block:: openrave.py --example %s --help

Main Python Code
----------------

.. literalinclude:: ../../../python/examples/%s.py
  :pyobject: main

Class Definitions
-----------------
"""%(self.arguments[0],self.arguments[0])
        parser = Parser()
        document = docutils.utils.new_document("<partial node>")
        document.settings = self.state.document.settings
        parser.parse(rawtext,document)
        return document.children

def setup(app):
    app.add_directive('examplepre-block', ExamplePreBlockDirective)
    app.add_directive('examplepost-block', ExamplePostBlockDirective)
