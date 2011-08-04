# generate a thumbnail gallery of examples
# code from matplotlib
template = """\
{%% extends "layout.html" %%}
{%% set title = "Database Generators" %%}


{%% block body %%}

%s

{%% endblock %%}
"""

import os, glob, re, sys, warnings
import openravepy
from openravepy.misc import mkdir_recursive
from types import ModuleType
import Image

def out_of_date(original, derived):
    return (not os.path.exists(derived) or os.stat(derived).st_mtime < os.stat(original).st_mtime)

def gen_gallery_internal(app, doctree,parentname,includedocstring,maxwidth,maxheight,maxcolumns):

    imagewritedir = os.path.join(app.builder.outdir,'_images')
    imagereaddir = os.path.join('images')
    imagelinkdir = '_images'
    linkdir = 'openravepy'
    imageext = 'jpg'
    link_template = """<td><p><b>%s</b></p><a href="%s"><img src="%s" border="0" class="thumbimage" alt="%s"/></a>%s</td>\n"""

    mkdir_recursive(os.path.join(imagewritedir,parentname))
    parentmodule = __import__('openravepy.'+parentname,fromlist=['openravepy'])
    modulenames = []
    for name in dir(parentmodule):
        if not name.startswith('__'):
            try:
                modulename = 'openravepy.'+parentname+'.'+name
                m=__import__(modulename,fromlist=['openravepy'])
                if type(m) is ModuleType:
                    docstring = ''
                    if m.__doc__ is not None and includedocstring:
                        endindex = m.__doc__.find('\n')
                        docstring = m.__doc__[:endindex] if endindex > 0 else ''
                    modulenames.append([modulename,name,docstring])
            except ImportError:
                pass

    rows = []
    for modulename, name, docstring in modulenames:
        imthumbname = name+'_thumb.'+imageext
        try:
            im = Image.open(os.path.join(imagereaddir,parentname,name+'_thumb.'+imageext))
        except IOError:
            try:
                im = Image.open(os.path.join(imagereaddir,parentname,name+'.'+imageext))
            except IOError:
                im = None
        if im is not None:
            if im.size[0]*maxheight/im.size[1] > maxwidth:
                newsize = [maxwidth,im.size[1]*maxwidth/im.size[0]]
            else:
                newsize = [im.size[0]*maxheight/im.size[1],maxheight]
            imthumb = im.resize(newsize, Image.ANTIALIAS)
            imthumb.save(open(os.path.join(imagewritedir,parentname,imthumbname),'w'))
        if len(docstring) > 0:
            docstring = '<p>%s</p>'%docstring
        rows.append(link_template%(name,linkdir+'/'+parentname+'.'+name+'.html', imagelinkdir+'/'+parentname+'/'+imthumbname, name,docstring))

    # Only write out the file if the contents have actually changed.
    # Otherwise, this triggers a full rebuild of the docs
    rowstext = ''
    for irow,row in enumerate(rows):
        if irow%maxcolumns == 0:
            rowstext += '<tr>'
        rowstext += row
        if irow%maxcolumns == maxcolumns-1:
            rowstext += '</tr>'
    if parentname == 'databases':
        rowstext = """
<h1>Database Generators</h1>

<p><a href="openravepy/openravepy.databases.html">Database Generators Overview</a></p>
<table border="1">
%s
</table>"""%rowstext
    elif parentname == 'examples':
        rowstext = """
<h1>Examples</h1>
<p><a href="getting_started.html">Getting Started Tutorials</a></p>
<p><a href="../coreapihtml/cpp_examples.html">C++ Examples Page</a></p>
<h3>Python Examples</h3>
<br/>

<table border="1">
%s
</table>

"""%rowstext

    content = template%rowstext

    gallery_path = os.path.join('_templates', parentname+'.html')
    if os.path.exists(gallery_path):
        fh = file(gallery_path, 'r')
        regenerate = fh.read() != content
        fh.close()
    else:
        regenerate = True
    if regenerate:
        fh = file(gallery_path, 'w')
        fh.write(content)
        fh.close()

def gen_gallery(app, doctree):
    if app.builder.name != 'html':
        return

    gen_gallery_internal(app,doctree,'databases',True,630,150,1)
    gen_gallery_internal(app,doctree,'examples',True,200,150,3)

def setup(app):
    app.connect('env-updated', gen_gallery)
