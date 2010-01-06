import os, weakref, inspect

class MetaInstanceTracker(type):
    def __init__(cls, name, bases, ns):
        super(MetaInstanceTracker, cls).__init__(name, bases, ns)
        cls.__instance_refs__ = []
    def __instances__(cls):
        instances = []
        validrefs = []
        for ref in cls.__instance_refs__:
            instance = ref()
            if instance is not None:
                instances.append(instance)
                validrefs.append(ref)
        cls.__instance_refs__ = validrefs
        return instances

class InstanceTracker(object):
    __metaclass__ = MetaInstanceTracker
    def __new__(*args, **kwargs):
        cls = args[0]
        self = super(InstanceTracker, cls).__new__(*args, **kwargs)
        cls.__instance_refs__.append(weakref.ref(self))
        return self

    def __reduce_ex__(self, proto):
        return super(InstanceTracker, self).__reduce_ex__(2)

class MetaAutoReloader(MetaInstanceTracker):
    def __init__(cls, name, bases, ns):
        super(MetaAutoReloader, cls).__init__(name, bases, ns)
        f = inspect.currentframe().f_back
        for d in [f.f_locals, f.f_globals]:
            if name in d:
                old_class = d[name]
                for instance in old_class.__instances__():
                    instance.change_class(cls)
                    cls.__instance_refs__.append(weakref.ref(instance))

                for subcls in old_class.__subclasses__():
                    newbases = []
                    for base in subcls.__bases__:
                        if base is old_class:
                            newbases.append(cls)
                        else:
                            newbases.append(base)
                    subcls.__bases__ = tuple(newbases)
                break

class AutoReloader(InstanceTracker):
    __metaclass__ = MetaAutoReloader
    def change_class(self, new_class):
        self.__class__ = new_class

def mkdir_recursive(newdir):
    """works the way a good mkdir should :)
        - already exists, silently complete
        - regular file in the way, raise an exception
        - parent directory(ies) does not exist, make them as well
    """
    if os.path.isdir(newdir):
        pass
    elif os.path.isfile(newdir):
        raise OSError("a file with the same name as the desired dir, '%s', already exists." % newdir)
    else:
        head, tail = os.path.split(newdir)
        if head and not os.path.isdir(head):
            mkdir_recursive(head)
        if tail:
            os.mkdir(newdir)
