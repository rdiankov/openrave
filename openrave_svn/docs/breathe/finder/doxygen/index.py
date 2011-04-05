
from breathe.finder.doxygen.base import ItemFinder 

class DoxygenTypeSubItemFinder(ItemFinder):

    def find(self, matcher):

        compounds = self.data_object.get_compound()

        results = []

        for compound in compounds:

            compound_finder = self.item_finder_factory.create_finder(compound)
            results.extend(compound_finder.find(matcher))

        return results

class CompoundTypeSubItemFinder(ItemFinder):

    def __init__(self, matcher_factory, compound_parser, *args):
        ItemFinder.__init__(self, *args)

        self.matcher_factory = matcher_factory
        self.compound_parser = compound_parser

    def find(self, matcher):

        members = self.data_object.get_member()

        member_results = []

        for member in members:
            member_finder = self.item_finder_factory.create_finder(member)
            member_results.extend(member_finder.find(matcher))

        results = []

        # If there are members in this compound that match the criteria 
        # then load up the file for this compound and get the member data objects
        if member_results:

            file_data = self.compound_parser.parse(self.data_object.refid)
            finder = self.item_finder_factory.create_finder(file_data)

            for member_data in member_results:
                ref_matcher = self.matcher_factory.create_ref_matcher(member_data.refid)
                results.extend(finder.find(matcher))

        elif matcher.match(self.data_object):
            results.append(self.data_object)

        return results

class MemberTypeSubItemFinder(ItemFinder):

    def find(self, matcher):

        if matcher.match(self.data_object):
            return [self.data_object]
        else:
            return []


