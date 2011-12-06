#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2010 Rosen Diankov (rosen.diankov@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os, sys
import fnmatch
from optparse import OptionParser
import breathe # for doxygen links
import sphinx
import docutils.nodes
from docutils.nodes import raw
from docutils.parsers import rst
from docutils.parsers.rst import Directive
import xml2rst
import StringIO
import re

class FunctionArgumentMatcher(breathe.finder.doxygen.ItemMatcher):
    """Matches the function name and its arguments.
    Can optionally specify a set of ids in which the matching function has be at least one of them
    """
    def __init__(self, name,ids=None):
        """Input the function name to match and create the cached data.

        :param name: Of the form **funcname "type1; type2; ..."**. For now using ';' for separating the arguments, but it can be easily changed to ','.
        """
        self.ids=None
        self.params = None
        paramindex = name.find('"')
        if paramindex>=0:
            endparamindex = name.rfind('"')
            if endparamindex >= 0:
                self.params = [s.strip() for s in name[(paramindex+1):endparamindex].split(';') if len(s.strip()) > 0]
            name = name[:paramindex]
        breathe.finder.doxygen.ItemMatcher.__init__(self,name.strip(),'function')
    def match(self, data_object):
        matches = False
        if self.name == data_object.name and self.type_ == data_object.kind:
            if hasattr(data_object,'get_refid'):
                if self.ids is None or data_object.get_refid() in self.ids:
                    matches = True
            elif hasattr(data_object,'get_id') and hasattr(data_object,'get_param'):
                if self.ids is None or data_object.get_id() in self.ids:
                    matches = True
                    if self.params is not None:
                        matches = False
                        if len(self.params) == len(data_object.get_param()):
                            # check if parameters are good
                            matches = True
                            for i in range(len(self.params)):
                                argtype = data_object.get_param()[i].get_type()
                                outfile=StringIO.StringIO()
                                argtype.exportChildren(outfile,0)
                                outfile.seek(0)
                                argtypetext = ''.join(s for s in outfile.read().strip() if s != '\n')
                                # clean out and <ref ..>X</ref> tags while preserving X
                                while True:
                                    index=argtypetext.find('<ref ')
                                    if index < 0:
                                        break
                                    endindex = argtypetext.find('</ref>',index+4)
                                    if endindex < 0:
                                        break
                                    startkeywordindex = argtypetext.rfind('>',index+4,endindex)
                                    if startkeywordindex < 0:
                                        break
                                    argtypetext = argtypetext[:index] + argtypetext[(startkeywordindex+1):endindex] + argtypetext[(endindex+6):]
                                if not argtypetext.startswith(self.params[i]):
                                    matches = False
                                    break
        return matches
    
    def __repr__(self):
        return "<MemberFunctionMatcher - name:%s>" % (self.name)

if __name__ == "__main__":
    parser = OptionParser(description="""Small utility that converts a set of doxygen comments to restructuredText for inclusion into python documentation.
Uses python docutils, sphinx, breathe, and xml2rst.""")
    parser.add_option('-i', action="store",type='string',dest='infile',default='../python/bindings/*.cpp',
                      help='Input C++ directory to look for DOXY_* tags to know what to generate (default=%default).')
    parser.add_option('-o', action="store",type='string',dest='outfile',default='../python/bindings/docstrings.cpp',
                      help='Output C++ files that offers the comments.')
    parser.add_option('--languagecode','-l', action="append",type='string',dest='languagecodes',default=[],
                      help='Add a language code, the doxygen directory that will be searched for is build/LANG/coreapixml.')
    (options, args) = parser.parse_args()
    if len(options.languagecodes) == 0:
        print 'need to specify at least one language code!'
        sys.exit(1)

    functions = []
    enums = []
    classes = []

    searchpath, searchstring = os.path.split(options.infile)
    for file in os.listdir(searchpath):
        if not fnmatch.fnmatch(file, searchstring):
            continue
        # brief+detailed descriptions only, do not include entire scope
        rawcppdata = open(os.path.join(searchpath,file),'r').read()
        functions_temp=re.findall('DOXY_FN\(\s*([\w:]*)\s*,([\s\w:;* <>&"]*)\)',rawcppdata)
        while len(functions_temp) > 0 and functions_temp[0][0] == 'class':
            functions_temp.pop(0) # remove the #define's
        functions += functions_temp
        # add all functions without member classes
        functions_temp=re.findall('DOXY_FN1\(([\s\w:;* <>&"]*)\)',rawcppdata)
        for name in functions_temp:
            if name != 'name':
                functions.append((None,name))
        enums_temp=re.findall('DOXY_ENUM\(\s*([\w:]*)\s*\)',rawcppdata)
        while len(enums_temp) > 0 and enums_temp[0] == 'name':
            enums_temp.pop(0) # remove the #define's
        enums += enums_temp
        classes_temp=re.findall('DOXY_CLASS\(\s*([\w:]*)\s*\)',rawcppdata)
        while len(classes_temp) > 0 and classes_temp[0] == 'name':
            classes_temp.pop(0) # remove the #define's
        classes += classes_temp

    xml2rst.setDefaultOptions()
    mainXsltF = open('xml2rst.xsl','r')
    projectname = 'openrave'
    defaultnamespace = 'OpenRAVE'

    # doxygen links using breathe
    parser_factory = breathe.DoxygenParserFactory()
    matcher_factory = breathe.ItemMatcherFactory()
    item_finder_factory_creator = breathe.DoxygenItemFinderFactoryCreator(parser_factory, matcher_factory)
    index_parser = breathe.DoxygenIndexParser()
    finder_factory = breathe.FinderFactory(index_parser, item_finder_factory_creator)
    node_factory = breathe.NodeFactory(docutils.nodes,sphinx.addnodes)
    renderer_factory_creator = breathe.DoxygenToRstRendererFactoryCreator(node_factory, parser_factory)
    project_info_factory = breathe.ProjectInfoFactory()
    builder_factory = breathe.BuilderFactory(breathe.RstBuilder, renderer_factory_creator)

    comments = ''
    for lang in options.languagecodes:
        project_info_factory.update({projectname:os.path.join('build',lang,'coreapixml')},projectname)
        project_info = project_info_factory.create_project_info({})
        finder = finder_factory.create_finder(project_info)

        # find the correct references
        comment_objects = []
        class_objects= []
        for class_name in classes:
            matcher = matcher_factory.create_name_type_matcher(class_name,'class')
            found_classes = finder.find(matcher)
            if len(found_classes) == 0:
                class_name = defaultnamespace+'::'+class_name
                matcher = matcher_factory.create_name_type_matcher(class_name,'class')
                found_classes = finder.find(matcher)
            if len(found_classes) > 0:
                #comment_objects.append(('%s class %s'%(lang,class_name),found_classes[0]))
                class_objects.append(found_classes[0])
            else:
                print 'failed to find class %s'%class_name

        for classname,functionname in functions:
            matcher = FunctionArgumentMatcher(functionname)
            matcher_basic = matcher_factory.create_name_type_matcher(matcher.name, 'function')
            data_object = None
            if classname is not None:
                ids = []
                # if a class specified, look for the one in the class
                for c in class_objects:
                    if c.name.endswith(classname):
                        data_objects = c.find_members(matcher_basic)
                        for d in data_objects:
                            ids.append(d.get_refid())
                if len(ids) > 0:
                    matcher.ids = ids
                else:
                    print 'class %s could not find member: %s: '%(classname,functionname)
            data_objects = list(set(finder.find(matcher))) # have to prune non-unique ones
            if len(data_objects) > 0:
                if len(data_objects) > 1:
                    print 'Function Overloaded: '+functionname
                data_object = data_objects[0]
            if data_object is not None:
                if classname is not None:
                    functionname = classname + ' ' + functionname
                comment_objects.append(('%s function %s'%(lang,functionname),data_object))
            else:
                print 'Cannot find "%s::%s" function in xml output' %(classname, functionname)

        for name in enums:
            matcher = matcher_factory.create_name_type_matcher(name,'enum')
            data_objects = finder.find(matcher)
            if len(data_objects)>0:
                comment_objects.append(('%s enum %s'%(lang,name),data_objects[0]))
            else:
                print 'Cannot find "%s" enum in xml output' %name
        
        # build the reStructuredText
        document = docutils.utils.new_document('.')
        builder = builder_factory.create_builder(project_info, document=document)
        for id, data_object in comment_objects:
            nodes = builder.build(data_object)
            #failed due to document.note_explicit_target(target) being called
            inF = StringIO.StringIO()
            outF = StringIO.StringIO()
            for node in nodes:
                if isinstance(node,docutils.nodes.target):
                    # perhaps should link to doxygen?
                    continue
                # have to replace pending links with doxygen links
                for n in node.traverse(sphinx.addnodes.pending_xref):
                    contnode = n[0].deepcopy()
                    newnode = None
                    typ = n['reftype']
                    target = n['reftarget']
                    if typ == 'ref':
                        if target.startswith('openrave'):
                            target = target[8:]
                        elif target.startswith('project0'):
                            target = target[8:]
                        if target.rfind('_') >= 0:
                            index = target.rfind('_')
                        else:
                            index = len(target)
                        refuri = 'html/' + target[:index]+'.html'
                        if os.path.isfile(os.path.join(lang,refuri)):
                            refuri = '../' + refuri + '#'+target[(index+2):]
                        else:
                            refuri = 'html/' + target+'.html'
                            if os.path.isfile(os.path.join(lang,refuri)):
                                refuri = '../' + refuri
                            else:
                                print 'could not find a valid url from '+target
                                refuri = None
                        if refuri is not None:
                            sectname = n.astext()
                            innernode = docutils.nodes.emphasis(sectname, sectname)
                            newnode = docutils.nodes.reference('', '')
                            newnode['refid'] = target
                            newnode['refuri'] = refuri
                            newnode.append(innernode)
                    if newnode is None:
                        print 'could not replace: ',n.asdom().toxml()
                        newnode = contnode
                    n.replace_self(newnode)
                domtree=node.asdom()
                domtree.writexml(inF,indent="", addindent="",newl="")
            mainXsltF.seek(0)
            inF.seek(0)
            xml2rst.convert(inF=inF,outF=outF,mainXsltF=mainXsltF)
            outF.seek(0)
            comment = outF.read()
            # make sure there are at most two consecutive new lines (otherwise reading gets difficult)
            comment=re.sub('[\n\s]*(\n[\s]*\n)','\g<1>',comment)
            # need to go through the C string parser..
            comment=re.sub('\\\\','\\\\\\\\',comment)
            comment=re.sub('\n','\\\\n',comment)
            comment=re.sub('\t','\\\\t',comment)
            comment=re.sub('"','\\\\"',comment)
            id=re.sub('"','\\\\"',id)
            comments += 'm["%s"] = "\\n\\n%s";\n'%(id,comment)
            inF.close()
            outF.close()

    open(options.outfile,'w').write("""// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/// Auto-generated using %s from %s file
#include "docstrings.h"
namespace openravepy {
void InitializeComments(std::map<std::string,std::string>& m)
{%s
}
}
"""%(sys.argv[0],options.infile,comments))
    print 'Finished writing to doc strings to %s'%options.outfile
    mainXsltF.close()
