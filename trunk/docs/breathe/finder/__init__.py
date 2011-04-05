
class FinderError(Exception):
    pass

class MultipleMatchesError(FinderError):
    pass

class NoMatchesError(FinderError):
    pass

class Matcher(object):
    pass


class Finder(object):

    def __init__(self, root, item_finder_factory):

        self._root = root
        self.item_finder_factory = item_finder_factory

    def find(self, matcher):

        item_finder = self.item_finder_factory.create_finder(self._root)

        return item_finder.find(matcher)


    def find_one(self, matcher):

        results = self.find(matcher)

        count = len(results)
        if count == 1:
            return results[0]
        elif count > 1:
            # Multiple matches can easily happen as same thing
            # can be present in both file and group sections
            return results[0]
        elif count < 1:
            raise NoMatchesError(matcher)


    def root(self):

        return self._root


class FinderFactory(object):

    def __init__(self, parser, item_finder_factory_creator):

        self.parser = parser
        self.item_finder_factory_creator = item_finder_factory_creator


    def create_finder(self, project_info):

        root = self.parser.parse(project_info)
        item_finder_factory = self.item_finder_factory_creator.create_factory(project_info)

        return Finder(root, item_finder_factory)



