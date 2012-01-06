"""relpath is not present in python 2.5 and below, so hold an implementation of it.
"""
try:
    from os.path import relpath
except ImportError:
    from posixpath import curdir, sep, pardir, join, abspath, commonprefix

    def relpath(path, start=curdir):
        """Return a relative version of a path"""
        if not path:
            raise ValueError("no path specified")
        start_list = abspath(start).split(sep)
        path_list = abspath(path).split(sep)
        # Work out how much of the filepath is shared by start and path.
        i = len(commonprefix([start_list, path_list]))
        rel_list = [pardir] * (len(start_list)-i) + path_list[i:]
        if not rel_list:
            return curdir
        return join(*rel_list)
