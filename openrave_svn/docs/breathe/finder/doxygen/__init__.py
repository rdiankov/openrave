
from breathe.finder.doxygen import index as indexfinder
from breathe.finder.doxygen import compound as compoundfinder

from breathe.parser.doxygen import index, compound
from breathe.finder import Matcher

class ItemMatcher(Matcher):

    def __init__(self, name, type_):
        self.name = name
        self.type_ = type_

    def match(self, data_object):

        return self.name == data_object.name and self.type_ == data_object.kind

    def __repr__(self):

        return "<ItemMatcher - name:%s, type_:%s>" % (self.name, self.type_)

class RefMatcher(Matcher):

    def __init__(self, refid):

        self.refid = refid

    def match(self, data_object):

        return self.refid == data_object.refid


class ItemMatcherFactory(Matcher):

    def create_name_type_matcher(self, name, type_):

        return ItemMatcher(name, type_)

    def create_ref_matcher(self, ref):

        return RefMatcher(ref)


class CreateCompoundTypeSubFinder(object):

    def __init__(self, parser_factory, matcher_factory):

        self.parser_factory = parser_factory
        self.matcher_factory = matcher_factory

    def __call__(self, project_info, *args):

        compound_parser = self.parser_factory.create_compound_parser(project_info)
        return indexfinder.CompoundTypeSubItemFinder(self.matcher_factory, compound_parser, project_info, *args)



class DoxygenItemFinderFactory(object):

    def __init__(self, finders, project_info):

        self.finders = finders
        self.project_info = project_info

    def create_finder(self, data_object):

        return self.finders[data_object.__class__](self.project_info, data_object, self)


class DoxygenItemFinderFactoryCreator(object):

    def __init__(self, parser_factory, matcher_factory):

        self.parser_factory = parser_factory
        self.matcher_factory = matcher_factory

    def create_factory(self, project_info):

        finders = {
            index.DoxygenTypeSub : indexfinder.DoxygenTypeSubItemFinder,
            index.CompoundTypeSub : CreateCompoundTypeSubFinder(self.parser_factory, self.matcher_factory),
            index.MemberTypeSub : indexfinder.MemberTypeSubItemFinder,
            compound.DoxygenTypeSub : compoundfinder.DoxygenTypeSubItemFinder,
            compound.compounddefTypeSub : compoundfinder.CompoundDefTypeSubItemFinder,
            compound.sectiondefTypeSub : compoundfinder.SectionDefTypeSubItemFinder,
            compound.memberdefTypeSub : compoundfinder.MemberDefTypeSubItemFinder,
            }

        return DoxygenItemFinderFactory(finders, project_info)



