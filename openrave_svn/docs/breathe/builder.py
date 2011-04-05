
class Builder(object):

    pass


class RstBuilder(Builder):

    def __init__(self, renderer_factory):

        self.renderer_factory = renderer_factory

    def build(self, data_object):

        object_renderer = self.renderer_factory.create_renderer(data_object)

        return object_renderer.render()


class BuilderFactory(object):

    def __init__(self, builder_class, renderer_factory_creator):

        self.builder_class = builder_class
        self.renderer_factory_creator = renderer_factory_creator

    def create_builder(self, project_info, document):

        renderer_factory = self.renderer_factory_creator.create_factory(project_info, document)

        return self.builder_class(renderer_factory)

