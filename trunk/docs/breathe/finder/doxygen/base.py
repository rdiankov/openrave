
class ItemFinder(object):

    def __init__(self, project_info, data_object, item_finder_factory):

        self.data_object = data_object
        self.item_finder_factory = item_finder_factory
        self.project_info = project_info


