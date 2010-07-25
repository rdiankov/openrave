
class Renderer(object):

    def __init__(self, project_info, data_object, renderer_factory, node_factory, document):

        self.project_info = project_info
        self.data_object = data_object
        self.renderer_factory = renderer_factory
        self.node_factory = node_factory
        self.document = document


