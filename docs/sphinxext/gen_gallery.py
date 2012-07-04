# generate a thumbnail gallery of examples
# code from matplotlib
# template = """\
# {%% extends "layout.html" %%}
# {%% set title = "Database Generators" %%}
# 
# 
# {%% block body %%}
# 
# %s
# 
# {%% endblock %%}
# """
import docutils.nodes
from docutils.parsers.rst import Directive, Parser
import os, glob, re, sys, warnings
import openravepy
from types import ModuleType
import Image

class GalleryDirective(Directive):
    required_arguments = 1  # examples, gallery
    optional_arguments = 3
    final_argument = False
    option_spec = {'image-width': int, 'image-height': int, 'columns':int}
    has_content = False

    @staticmethod
    def out_of_date(original, derived):
        return (not os.path.exists(derived) or os.stat(derived).st_mtime < os.stat(original).st_mtime)

    def run(self):
        gallerytype = self.arguments[0]
        includedocstring = True
        maxwidth = self.options.get('image-width',630 if gallerytype == 'databases' else 200)
        maxheight = self.options.get('image-height',150)
        maxcolumns = self.options.get('columns',1 if gallerytype == 'databases' else 3)

        #self.state.document.settings.env.images
        #builder=self.state.document.settings.env.app.builder
        buildertype = self.state.document.settings.env.app.builder.name
        outdir = self.state.document.settings.env.app.builder.outdir
        #docname=self.state.document.settings.env.docname
        imagewritedir = os.path.join(os.path.join(outdir,'_images'),gallerytype)
        imagereaddir = os.path.join(self.state.document.settings.env.srcdir,'..','images',gallerytype)
        imagelinkdir = '_images'
        linkdir = 'openravepy'
        imageext = 'jpg'

        try:
            os.makedirs(imagewritedir)
        except OSError:
            pass
        parentmodule = __import__('openravepy.'+gallerytype,fromlist=['openravepy'])
        modulenames = []
        for name in dir(parentmodule):
            if not name.startswith('__'):
                try:
                    modulename = 'openravepy.'+gallerytype+'.'+name
                    m=__import__(modulename,fromlist=['openravepy'])
                    if type(m) is ModuleType:
                        docstring = ''
                        if m.__doc__ is not None and includedocstring:
                            endindex = m.__doc__.find('\n')
                            docstring = m.__doc__[:endindex] if endindex > 0 else ''
                        modulenames.append([modulename,name,docstring])
                except ImportError:
                    pass

        # copy the images
        link_templates = {'html':'<td><p><b>%s</b></p><a href="%s.html"><img src="%s" border="0" class="thumbimage" alt="%s"/></a>%s</td>\n', 'json':'<td><p><b>%s</b></p><a href="../%s/"><img src="../%s" border="0" class="thumbimage" alt="../%s"/></a>%s</td>\n'}
        link_template = link_templates.get(buildertype,link_templates['html'])
        rows = []
        for modulename, name, docstring in modulenames:
            imthumbname = name+'_thumb.'+imageext
            try:
                im = Image.open(os.path.join(imagereaddir,name+'_thumb.'+imageext))
            except IOError:
                try:
                    im = Image.open(os.path.join(imagereaddir,name+'.'+imageext))
                except IOError:
                    im = None
            if im is not None:
                if im.size[0]*maxheight/im.size[1] > maxwidth:
                    newsize = [maxwidth,im.size[1]*maxwidth/im.size[0]]
                else:
                    newsize = [im.size[0]*maxheight/im.size[1],maxheight]
                imthumb = im.resize(newsize, Image.ANTIALIAS)
                imthumb.save(open(os.path.join(imagewritedir,imthumbname),'w'))
                if len(docstring) > 0:
                    docstring = '<p>%s</p>'%docstring
                rows.append(link_template%(name,linkdir+'/'+gallerytype+'.'+name, imagelinkdir+'/'+gallerytype+'/'+imthumbname, name,docstring))
                    
        # have to have different links for different builders
        #for buildertype,link_template in link_templates.iteritems():
# 
#             for modulename, name, docstring in modulenames:
#                 imthumbname = name+'_thumb.'+imageext


            # Only write out the file if the contents have actually changed.
            # Otherwise, this triggers a full rebuild of the docs
        rowstext = '<table>'
        for irow,row in enumerate(rows):
            if irow%maxcolumns == 0:
                rowstext += '<tr>'
            rowstext += row
            if irow%maxcolumns == maxcolumns-1:
                rowstext += '</tr>\n'
        rowstext += '</table>'
        # add two spaces for every new line since using htmlonly tag
        content = '.. raw:: html\n\n  '+rowstext.replace('\n','\n    ')+'\n\n'
            
        parser = Parser()
        document = docutils.utils.new_document("<partial node>")
        document.settings = self.state.document.settings
        parser.parse(content,document)
        return document.children

def setup(app):
    app.add_directive('gallery', GalleryDirective)
