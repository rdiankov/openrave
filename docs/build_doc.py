#!/usr/bin/env python
# -*- coding: utf-8 -*-
from distutils.cmd import Command
import doctest
from glob import glob
import os
import sys
import breathe # for doxygen links
import sphinx # for better rst docs? needed by breathe
try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup
import StringIO, shlex, subprocess # for shell commands
import re

class build_doc(Command):
    description = 'Builds the OpenRAVE python documentation'
    user_options = [
        ('force', None,
         "force regeneration even if no reStructuredText files have changed"),
        ('without-apidocs', None,
         "whether to skip the generation of API documentaton"),
        ('outdir=',None,
         "output dir of the document"),
        ('languagecode=',None,
         "language code to compile text with")
    ]
    boolean_options = ['force', 'without-apidocs']

    def initialize_options(self):
        self.force = False
        self.without_apidocs = False
        self.outdir = None
        self.languagecode = None

    def finalize_options(self):
        if self.outdir is None:
            self.outdir = 'openravepy-html'
        if self.languagecode is None:
            self.languagecode = 'en'

    def run(self):
        languagecode = self.languagecode
        from docutils.core import publish_cmdline
        import docutils.nodes
        from docutils.nodes import raw
        from docutils.parsers import rst
        from docutils.parsers.rst import Directive
        import __builtin__
        __builtin__.__openravepy_build_doc__ = True # notify openravepy that will be building docs so it can re-arrange its imports

        docutils_conf = os.path.abspath('docutils.ini')
        epydoc_conf = os.path.abspath('epydoc.config')
        epydoc_css = os.path.abspath('epydoc.css')
        epydoc_out = os.path.abspath(self.outdir)

        try:
            from pygments import highlight
            from pygments.lexers import get_lexer_by_name
            from pygments.formatters import HtmlFormatter

            def code_block(name, arguments, options, content, lineno, content_offset, block_text, state, state_machine):
                lexer = get_lexer_by_name(arguments[0])
                html = highlight('\n'.join(content), lexer, HtmlFormatter())
                return [raw('', html, format='html')]
            code_block.arguments = (1, 0, 0)
            code_block.options = {'language' : rst.directives.unchanged}
            code_block.content = 1
            rst.directives.register_directive('code-block', code_block)
        except ImportError:
            print 'Pygments not installed, syntax highlighting disabled'

        # register a custom language handler
        class LangBlockDirective(Directive):
            required_arguments = 1
            optional_arguments = 0
            final_argument = False
            option_spec = {'language':rst.directives.unchanged}
            has_content = True
            node_class = rst.nodes.paragraph
            def run(self):
                self.assert_has_content()
                if self.arguments[0] != languagecode:
                    return [] # does not match language code, so ignore
                text = '\n'.join(self.content)
                node = self.node_class(text)
                #node['classes'] += self.options.get('class', [])
                if text:
                    self.state.nested_parse(self.content, self.content_offset, node)
                return [node]
        rst.directives.register_directive('lang-block', LangBlockDirective)

        class InterfaceCommandDirective(Directive):
            required_arguments = 2
            optional_arguments = 0
            final_argument = False
            option_spec = {'interface':rst.directives.unchanged,'command':rst.directives.unchanged}
            has_content = True
            node_class = rst.nodes.paragraph
            def run(self):
                return []
        rst.directives.register_directive('interface-command', InterfaceCommandDirective)

        class ShellCommandDirective(Directive):
            required_arguments = 0
            optional_arguments = 0
            final_argument = False
            option_spec = {}
            has_content = True
            def run(self):
                self.assert_has_content()
                args = shlex.split(self.content[0].encode('ascii'))
                text = subprocess.Popen(args,stdout=subprocess.PIPE).communicate()[0]
                return [docutils.nodes.literal_block(text=text)]
        rst.directives.register_directive('shell-block', ShellCommandDirective)

        # doxygen links using breathe
        parser_factory = breathe.DoxygenParserFactory()
        matcher_factory = breathe.ItemMatcherFactory()
        item_finder_factory_creator = breathe.DoxygenItemFinderFactoryCreator(parser_factory, matcher_factory)
        index_parser = breathe.DoxygenIndexParser()
        finder_factory = breathe.FinderFactory(index_parser, item_finder_factory_creator)
        node_factory = breathe.NodeFactory(docutils.nodes,sphinx.addnodes)
        renderer_factory_creator = breathe.DoxygenToRstRendererFactoryCreator(node_factory, parser_factory)
        builder_factory = breathe.BuilderFactory(breathe.RstBuilder, renderer_factory_creator)
        project_info_factory = breathe.ProjectInfoFactory()
        directive_factory = breathe.DoxygenDirectiveFactory(builder_factory, finder_factory, matcher_factory, project_info_factory)
        project_info_factory.update({'openrave':os.path.join(languagecode, 'xml')},'openrave')
        rst.directives.register_directive("doxygenindex", directive_factory.create_index_directive_container())
        rst.directives.register_directive("doxygenfunction", directive_factory.create_function_directive_container())
        rst.directives.register_directive("doxygenstruct", directive_factory.create_struct_directive_container())
        rst.directives.register_directive("doxygenenum",directive_factory.create_enum_directive_container())
        rst.directives.register_directive("doxygentypedef",directive_factory.create_typedef_directive_container())
        rst.directives.register_directive("doxygenclass",directive_factory.create_class_directive_container())

        rst.languages.get_language('ja')

        for source in glob('*.txt'):
            dest = os.path.splitext(source)[0] + '.html'
            if self.force or not os.path.exists(dest) or \
                    os.path.getmtime(dest) < os.path.getmtime(source):
                print 'building documentation file %s' % dest
                publish_cmdline(writer_name='html',
                                argv=['--config=%s' % docutils_conf, source,
                                      dest])

        if not self.without_apidocs:
            try:
                from epydoc import cli
                from epydoc import docbuilder
                from epydoc import docintrospecter

                # override the default html translator to support non-default tags (from sphinx)
                from epydoc.markup import restructuredtext
                class NewEpydocHTMLTranslator(restructuredtext._EpydocHTMLTranslator):
                    def visit_pending_xref(self, node):
                        typ = node['reftype']
                        target = node['reftarget']
                        if typ == 'ref':
                            if target.rfind('_') >= 0:
                                if target.startswith('openrave'):
                                    target = target[8:]
                                elif target.startswith('project0'):
                                    target = target[8:]
                                index = target.rfind('_')
                                refuri = '../html/' + target[:index]+'.html#'+target[(index+2):]
                                self.body.append('<a href="%s">%s</a>'%(refuri,node.astext()))
                                raise docutils.nodes.SkipNode()
                    def depart_pending_xref(self, node):
                        pass
                    def visit_warning(self,node):
                        pass
                    def depart_warning(self,node):
                        pass
                restructuredtext._EpydocHTMLTranslator = NewEpydocHTMLTranslator
                
                # have to set default encoding to utf-8
                epydoc_get_docstring = docintrospecter.get_docstring
                def new_get_docstring(value, module_name=None):
                    if module_name is None and docintrospecter.get_containing_module(value) is None:
                        # default to openravepy in order to get utf-8!
                        module_name='openravepy'
                    return epydoc_get_docstring(value, module_name)
                docintrospecter.get_docstring = new_get_docstring

                old_argv = sys.argv[1:]
                sys.argv[1:] = [
                    '--config=%s' % epydoc_conf,
                    '--css=%s'%epydoc_css,
                    '--output=%s'%epydoc_out,
                    '--verbose'
                ]
                cli.cli()
                sys.argv[1:] = old_argv
            except ImportError:
                print 'epydoc not installed, skipping API documentation.'


class test_doc(Command):
    description = 'Tests the code examples in the documentation'
    user_options = []
    def initialize_options(self):
        pass
    def finalize_options(self):
        pass
    def run(self):
        for filename in glob('*.txt'):
            print 'testing documentation file %s' % filename
            doctest.testfile(filename, False, optionflags=doctest.ELLIPSIS)

setup(cmdclass={'build_doc': build_doc, 'test_doc': test_doc})

# requirements = []
# 
# setup(
#     name = 'OpenRAVE',
#     version = '0.2.2',
#     description = 'OpenRAVE Python bindings',
#     long_description = """Open Robotics Automation Virtual Environment""",
#     author = 'Rosen Diankov',
#     author_email = 'rosen.diankov@gmail.com',
#     license = 'LGPL + Apache License, Version 2.0',
#     url = 'http://openrave.programmingvision.com/',
#     zip_safe = True,
# 
#     classifiers = [
#     ],
#     packages = ['openravepy', 'openravepy.examples', 'openravepy.databases'],
#     test_suite = 'openravepy.tests',
# 
#     install_requires = requirements,
# 
#     entry_points = {
#         'console_scripts': None,
#     },
# 
#     cmdclass = {'build_doc': build_doc, 'test_doc': test_doc}
# )
