
from breathe.renderer.rst.doxygen.base import Renderer
from breathe.renderer.rst.doxygen import index as indexrenderer
from breathe.renderer.rst.doxygen import compound as compoundrenderer

from breathe.parser.doxygen import index, compound, compoundsuper


class UnicodeRenderer(Renderer):

    def render(self):

        return [self.node_factory.Text(self.data_object)]

class DoxygenToRstRendererFactory(object):

   def __init__(self, renderers, node_factory, project_info, document):

        self.node_factory = node_factory
        self.project_info = project_info
        self.renderers = renderers
        self.document = document

   def create_renderer(self, data_object):

        return self.renderers[data_object.__class__](
                self.project_info,
                data_object,
                self,
                self.node_factory,
                self.document
                )


class CreateCompoundTypeSubRenderer(object):

    def __init__(self, parser_factory):

        self.parser_factory = parser_factory

    def __call__(self, project_info, *args):

        compound_parser = self.parser_factory.create_compound_parser(project_info)
        return indexrenderer.CompoundTypeSubRenderer(compound_parser, project_info, *args)


class DoxygenToRstRendererFactoryCreator(object):

    def __init__(self, node_factory, parser_factory):

        self.node_factory = node_factory
        self.parser_factory = parser_factory

    def create_factory(self, project_info, document):

        renderers = {
            index.DoxygenTypeSub : indexrenderer.DoxygenTypeSubRenderer,
            index.CompoundTypeSub : CreateCompoundTypeSubRenderer(self.parser_factory),
            compound.DoxygenTypeSub : compoundrenderer.DoxygenTypeSubRenderer,
            compound.compounddefTypeSub : compoundrenderer.CompoundDefTypeSubRenderer,
            compound.sectiondefTypeSub : compoundrenderer.SectionDefTypeSubRenderer,
            compound.memberdefTypeSub : compoundrenderer.MemberDefTypeSubRenderer,
            compound.linkedTextTypeSub : compoundrenderer.LinkedTextTypeSubRenderer,
            compound.descriptionTypeSub : compoundrenderer.DescriptionTypeSubRenderer,
            compound.paramTypeSub : compoundrenderer.ParamTypeSubRenderer,
            compound.docRefTextTypeSub : compoundrenderer.DocRefTextTypeSubRenderer,
            compound.docParaTypeSub : compoundrenderer.DocParaTypeSubRenderer,
            compound.docParamListTypeSub : compoundrenderer.DocParamListTypeSubRenderer,
            compound.docParamListItemSub : compoundrenderer.DocParamListItemSubRenderer,
            compound.docParamNameListSub : compoundrenderer.DocParamNameListSubRenderer,
            compound.docParamNameSub : compoundrenderer.DocParamNameSubRenderer,
            compound.docSect1TypeSub : compoundrenderer.DocSect1TypeSubRenderer,
            compound.docSimpleSectTypeSub : compoundrenderer.DocSimpleSectTypeSubRenderer,
            compoundsuper.MixedContainer : compoundrenderer.MixedContainerRenderer,
            unicode : UnicodeRenderer,
            }

        return DoxygenToRstRendererFactory(renderers, self.node_factory, project_info, document)


