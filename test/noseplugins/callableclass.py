"""If a class provides a __call__ method, then it treats the class as a regular function call that is passed arguments. Enable with --with-callableclass

"""
import sys
import nose
from nose.plugins.base import Plugin
import inspect, types
__author__='Rosen Diankov'

class CallableClass(Plugin):
    name = 'callableclass'
    score = 2000
    def makeTest(self, obj, parent=None):
        if inspect.isclass(obj) and callable(obj):
            if parent and not isinstance(parent, types.ModuleType):
                # treat class as a function call
                obj = unbound_method(parent, obj)
            if parent and obj.__module__ != parent.__name__:
                obj = transplant_func(obj, parent.__name__)
            if inspect.isgenerator(obj):
                return self.loadTestsFromGenerator(obj, parent)
            else:
                return [nose.case.FunctionTestCase(obj())]
        return None
