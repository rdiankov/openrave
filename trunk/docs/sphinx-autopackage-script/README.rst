
Generate Modules
================

This script parses a directory tree looking for python modules and packages and
creates ReST files appropriately to create code documentation with Sphinx.
It also creates a modules index. 


Usage::

    Usage: generate_modules.py [options] <package path> [exclude paths, ...]
        
    Note: By default this script will not overwrite already created files.
    
    Options:
      -h, --help            show this help message and exit
      -n HEADER, --doc-header=HEADER
                            Documentation Header (default=Project)
      -d DESTDIR, --dest-dir=DESTDIR
                            Output destination directory
      -s SUFFIX, --suffix=SUFFIX
                            module suffix (default=txt)
      -m MAXDEPTH, --maxdepth=MAXDEPTH
                            Maximum depth of submodules to show in the TOC
                            (default=4)
      -r, --dry-run         Run the script without creating the files
      -f, --force           Overwrite all the files
      -t, --no-toc          Don't create the table of content file
