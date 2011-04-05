
from breathe.finder.doxygen.base import ItemFinder 

class DoxygenTypeSubItemFinder(ItemFinder):

    def find(self, matcher):

        compound_finder = self.item_finder_factory.create_finder(self.data_object.compounddef)
        return compound_finder.find(matcher)

class CompoundDefTypeSubItemFinder(ItemFinder):

    def find(self, matcher):

        results = []
        for sectiondef in self.data_object.sectiondef:
            finder = self.item_finder_factory.create_finder(sectiondef)
            results.extend(finder.find(matcher))

        return results

        
class SectionDefTypeSubItemFinder(ItemFinder):

    def find(self, matcher):

        results = []
        for memberdef in self.data_object.memberdef:
            finder = self.item_finder_factory.create_finder(memberdef)
            results.extend(finder.find(matcher))

        return results

class MemberDefTypeSubItemFinder(ItemFinder):

    def find(self, matcher):

        if matcher.match(self.data_object):
            return [self.data_object]
        else:
            return []

