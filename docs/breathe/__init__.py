
from docutils import nodes
from docutils.parsers.rst.directives import unchanged_required

import os
import sys
import copy

from docutils.parsers import rst
from breathe.builder import RstBuilder, BuilderFactory
from breathe.finder import FinderFactory, NoMatchesError, MultipleMatchesError
from breathe.parser import DoxygenParserFactory, DoxygenIndexParser
from breathe.renderer.rst.doxygen import DoxygenToRstRendererFactoryCreator
from breathe.finder.doxygen import DoxygenItemFinderFactoryCreator, ItemMatcherFactory

import docutils.nodes
import sphinx.addnodes

class BaseDirective(rst.Directive):

    def __init__(self, builder_factory, finder_factory, matcher_factory, project_info_factory, *args):
        rst.Directive.__init__(self, *args)

        self.builder_factory = builder_factory
        self.finder_factory = finder_factory
        self.matcher_factory = matcher_factory
        self.project_info_factory = project_info_factory


# Directives
# ----------

class DoxygenIndexDirective(BaseDirective):

    required_arguments = 0
    optional_arguments = 2
    option_spec = {
            "path" : unchanged_required,
            "project" : unchanged_required,
            }
    has_content = False

    def run(self):

        project_info = self.project_info_factory.create_project_info(self.options)

        finder = self.finder_factory.create_finder(project_info)

        # try:
        data_object = finder.root()
        # except

        builder = self.builder_factory.create_builder(project_info, self.state.document)
        nodes = builder.build(data_object)

        return nodes

class DoxygenFunctionDirective(BaseDirective):

    required_arguments = 1
    optional_arguments = 1
    option_spec = {
            "path" : unchanged_required,
            "project" : unchanged_required,
            }
    has_content = False

    def run(self):

        function_name = self.arguments[0]

        project_info = self.project_info_factory.create_project_info(self.options)

        finder = self.finder_factory.create_finder(project_info)

        matcher = self.matcher_factory.create_name_type_matcher(function_name, "function")

        try:
            data_object = finder.find_one(matcher)
        except NoMatchesError, e:
            warning = 'doxygenfunction: Cannot find function "%s" in doxygen xml output' % function_name
            return [ docutils.nodes.warning( "", docutils.nodes.paragraph("", "", docutils.nodes.Text(warning))),
                    self.state.document.reporter.warning( warning, line=self.lineno) ]



        builder = self.builder_factory.create_builder(project_info, self.state.document)
        nodes = builder.build(data_object)

        return nodes



class DoxygenStructDirective(BaseDirective):

    kind = "struct"

    required_arguments = 1
    optional_arguments = 1
    option_spec = {
            "path" : unchanged_required,
            "project" : unchanged_required,
            }
    has_content = False

    def run(self):

        struct_name = self.arguments[0]

        project_info = self.project_info_factory.create_project_info(self.options)

        finder = self.finder_factory.create_finder(project_info)

        # try:
        matcher = self.matcher_factory.create_name_type_matcher(struct_name, self.kind)

        try:
            data_object = finder.find_one(matcher)
        except NoMatchesError, e:
            warning = 'doxygen%s: Cannot find %s "%s" in doxygen xml output' % (self.kind, self.kind, struct_name)
            return [ docutils.nodes.warning( "", docutils.nodes.paragraph("", "", docutils.nodes.Text(warning))),
                    self.state.document.reporter.warning( warning, line=self.lineno) ]

        builder = self.builder_factory.create_builder(project_info, self.state.document)
        nodes = builder.build(data_object)

        return nodes


class DoxygenClassDirective(DoxygenStructDirective):

    kind = "class"


class DoxygenEnumDirective(DoxygenStructDirective):

    kind = "enum"


class DoxygenTypedefDirective(DoxygenStructDirective):

    kind = "typedef"



# Setup Administration
# --------------------

class DirectiveContainer(object):

    def __init__(self, directive, builder, finder_factory, matcher_factory, project_info_factory):

        self.directive = directive
        self.builder = builder
        self.finder_factory = finder_factory
        self.matcher_factory = matcher_factory
        self.project_info_factory = project_info_factory

        # Required for sphinx to inspect
        self.required_arguments = directive.required_arguments
        self.optional_arguments = directive.optional_arguments
        self.option_spec = directive.option_spec
        self.has_content = directive.has_content

    def __call__(self, *args):

        return self.directive(self.builder, self.finder_factory, self.matcher_factory, self.project_info_factory, *args)


class ProjectInfo(object):

    def __init__(self, name, path):

        self._name = name
        self._path = path

    def name(self):
        return self._name

    def path(self):
        return self._path

class ProjectInfoFactory(object):

    def __init__(self):

        self.projects = {}
        self.default_project = None

        self.project_count = 0
        self.project_info_store = {}

    def update(self, projects, default_project):

        self.projects = projects
        self.default_project = default_project

    def default_path(self):

        return self.projects[self.default_project]

    def create_project_info(self, options):

        name = ""
        path = self.default_path()

        if options.has_key("project"):
            try:
                path = self.projects[ options["project"] ]
                name = options["project"]
            except KeyError, e:
                sys.stderr.write(
                        "Unable to find project '%s' in breathe_projects dictionary" % options["project"]
                        )

        if options.has_key("path"):
            path = options["path"]


        try:
            return self.project_info_store[path]
        except KeyError:
            if not name:
                name = "project%s" % self.project_count
                self.project_count += 1

            project_info = ProjectInfo(name, path)

            self.project_info_store[path] = project_info

            return project_info



class DoxygenDirectiveFactory(object):

    directives = {
            "doxygenindex" : DoxygenIndexDirective,
            "doxygenfunction" : DoxygenFunctionDirective,
            "doxygenstruct" : DoxygenStructDirective,
            "doxygenclass" : DoxygenClassDirective,
            "doxygenenum" : DoxygenEnumDirective,
            "doxygentypedef" : DoxygenTypedefDirective,
            }

    def __init__(self, builder_factory, finder_factory, matcher_factory, project_info_factory):
        self.builder_factory = builder_factory
        self.finder_factory = finder_factory
        self.matcher_factory = matcher_factory
        self.project_info_factory = project_info_factory

    def create_index_directive_container(self):
        return self.create_directive_container("doxygenindex")

    def create_function_directive_container(self):
        return self.create_directive_container("doxygenfunction")

    def create_struct_directive_container(self):
        return self.create_directive_container("doxygenstruct")

    def create_enum_directive_container(self):
        return self.create_directive_container("doxygenenum")

    def create_typedef_directive_container(self):
        return self.create_directive_container("doxygentypedef")

    def create_class_directive_container(self):
        return self.create_directive_container("doxygenclass")

    def create_directive_container(self, type_):

        return DirectiveContainer(
                self.directives[type_],
                self.builder_factory,
                self.finder_factory,
                self.matcher_factory,
                self.project_info_factory
                )

    def get_config_values(self, app):

        # All DirectiveContainers maintain references to this project info factory
        # so we can update this to update them
        self.project_info_factory.update(
                app.config.breathe_projects,
                app.config.breathe_default_project
                )

class NodeFactory(object):

    def __init__(self, *args):

        self.sources = args

    def __getattr__(self, node_name):

        for source in self.sources:
            try:
                return getattr(source, node_name)
            except AttributeError:
                pass

        raise NodeNotFoundError(node_name)


# Setup
# -----

def setup(app):

    parser_factory = DoxygenParserFactory()
    matcher_factory = ItemMatcherFactory()
    item_finder_factory_creator = DoxygenItemFinderFactoryCreator(parser_factory, matcher_factory)
    index_parser = DoxygenIndexParser()
    finder_factory = FinderFactory(index_parser, item_finder_factory_creator)

    node_factory = NodeFactory(docutils.nodes, sphinx.addnodes)
    renderer_factory_creator = DoxygenToRstRendererFactoryCreator(node_factory, parser_factory)
    builder_factory = BuilderFactory(RstBuilder, renderer_factory_creator)

    project_info_factory = ProjectInfoFactory()
    directive_factory = DoxygenDirectiveFactory(builder_factory, finder_factory, matcher_factory, project_info_factory)

    app.add_directive(
            "doxygenindex",
            directive_factory.create_index_directive_container(),
            )

    app.add_directive(
            "doxygenfunction",
            directive_factory.create_function_directive_container(),
            )

    app.add_directive(
            "doxygenstruct",
            directive_factory.create_struct_directive_container(),
            )

    app.add_directive(
            "doxygenenum",
            directive_factory.create_enum_directive_container(),
            )

    app.add_directive(
            "doxygentypedef",
            directive_factory.create_typedef_directive_container(),
            )

    app.add_directive(
            "doxygenclass",
            directive_factory.create_class_directive_container(),
            )

    app.add_config_value("breathe_projects", {}, True)
    app.add_config_value("breathe_default_project", "", True)

    app.connect("builder-inited", directive_factory.get_config_values)


