#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
"""Imports a robot using python occ (tested v 0.5) library
"""
from __future__ import absolute_import

from OCC.XCAFApp import *
from OCC.STEPCAFControl import *
from OCC.TDocStd import *
from OCC.TCollection import *
from OCC.XCAFDoc import *
from OCC.TDF import *

from OCC.BRep import *
from OCC.BRepPrimAPI import *
from OCC.BRepAlgoAPI import *
from OCC.BRepBuilderAPI import *
from OCC.BRepMesh import *
from OCC.TopExp import *
from OCC.TopAbs import *
from OCC.TopoDS import *
from OCC.TopLoc import *
from OCC.Poly import *
from OCC.TColgp import *

from OCC import XCAFApp, TDocStd, TCollection, XCAFDoc, BRepPrimAPI, Quantity, TopLoc, gp,TPrsStd, XCAFPrs, STEPCAFControl, TDataStd, TNaming, TFunction, TPrsStd

from optparse import OptionParser
import os, sys
from openravepy import metaclass
from openravepy.misc import mkdir_recursive

from enthought.traits.api import (HasTraits, Property, Bool,  on_trait_change, cached_property, Instance, File, Float as _Float, List, Str, Enum, Int)

from openravepy import *
import numpy
import struct, ctypes

def GID2Tuple(gid):
    return (gid._CSFDB_GetStandard_GUIDmy32b(),
            gid._CSFDB_GetStandard_GUIDmy16b1(),
            gid._CSFDB_GetStandard_GUIDmy16b2(),
            gid._CSFDB_GetStandard_GUIDmy16b3(),
            gid._CSFDB_GetStandard_GUIDmy8b1(),
            gid._CSFDB_GetStandard_GUIDmy8b2(),
            gid._CSFDB_GetStandard_GUIDmy8b3(),
            gid._CSFDB_GetStandard_GUIDmy8b4(),
            gid._CSFDB_GetStandard_GUIDmy8b5(),
            gid._CSFDB_GetStandard_GUIDmy8b6())

def get_AttributeMap():
    modules = [TDataStd, TNaming, TDocStd, TFunction, TPrsStd]
    def iterAttributes():
        for m in modules:
            objItr = (getattr(m, n) for n in dir(m))
            for o in objItr:
                try:
                    if issubclass(o, Handle_TDF_Attribute):
                        yield o
                except TypeError:
                    pass
    attrList = list(iterAttributes())
    def get_id(c):
        try:
            return GID2Tuple(c().GetObject().GetID()), c
        except AttributeError:
            return None
    idList = (get_id(c) for c in attrList)
    filtered = (a for a in idList if a is not None)
    return dict(filtered)

AttributeMap = get_AttributeMap()

class Attribute(HasTraits):
    TDF_Attribute = Instance(TDF_Attribute)
    attr_cls = Property(depends_on="TDF_Attribute")
    cls_name = Property(Str, depends_on="TDF_Attribute")
    name = Property(Str, depends_on="attr_cls")
    
    def _get_attr_cls(self):
        ID = self.TDF_Attribute.ID()
        try:
            return AttributeMap[GID2Tuple(ID)]
        except KeyError:
            return None
        
    def Downcast(self):
        cls = self.attr_cls
        if cls is not None:
            other = cls()
            h_a = self.TDF_Attribute.GetHandle()
            new_h = other.DownCast(h_a)
            new_attr = new_h.GetObject()
            return new_attr
        
    def _get_cls_name(self):
        a = self.TDF_Attribute
        h = a.DynamicType()
        s = h.GetObject()
        return s.Name()
        
    def _get_name(self):
        try:
            a = self.Downcast()
            name = TCollection.TCollection_AsciiString(a.Get())
            return name.ToCString()
        except:
            return "no name"
        
class Label(HasTraits):
    TDF_Label = Instance(TDF_Label)
    children = Property(List, depends_on="sublabels, attributes")
    sublabels = Property(List, depends_on="TDF_Label")
    attributes = List(Attribute)
    entry = Str("unknown")
    
    name_attr = Property(Str, depends_on="TDF_Label")
    
    repr = Property(Str, depends_on="name_attr, entry")
    
    def _get_repr(self):
        return self.entry + " " + self.name_attr
    
    def _get_name_attr(self):
        label = self.TDF_Label
        h_a = TDataStd.Handle_TDataStd_Name()
        gid = TDataStd.TDataStd_Name().ID()
        if label.FindAttribute(gid, h_a):
            a = h_a.GetObject()
            name = TCollection.TCollection_AsciiString(a.Get())
            return name.ToCString()
        else:
            return "no name"
    
    def _get_children(self):
        return self.attributes + self.sublabels
    
    def _TDF_Label_changed(self, new_label):
        tool = TDF_Tool()
        output = TCollection.TCollection_AsciiString()
        tool.Entry(new_label, output)
        entry = output.ToCString()
        self.entry = entry
        
        itr = TDF_AttributeIterator(new_label, False)
        attrs = []
        while itr.More():
            val = itr.Value()
            #print val
            a = Attribute(TDF_Attribute=val)
            #print val, "->", a.Downcast()
            attrs.append(a)
            itr.Next()
        self.attributes = attrs
    
    def _get_sublabels(self):
        itr = TDF_ChildIterator(self.TDF_Label, False)
        result = []
        while itr.More():
            val = itr.Value()
            result.append(Label(TDF_Label=val))
            itr.Next()
        return result

class RobotImporter(metaclass.AutoReloader):
    class TriangleShape(object):
        __slots__=['name','trans','vertices','indices','color']
        

    def __init__(self,filename):
        self.reader = STEPCAFControl_Reader()
        self.reader.ReadFile(filename)
        #
        # Create the TDocStd document
        #
        self.h_doc = Handle_TDocStd_Document()

        #
        # Create the application: really *awfull* way to do that
        #
        self.app = GetApplication().GetObject()
        self.app.NewDocument(TCollection_ExtendedString("MDTV-CAF"),self.h_doc)
        self.orenv=None
        #
        # First open and parse STEP file
        #
        #
        # Transfer
        #
        self.reader.SetColorMode(True)
        self.reader.SetNameMode(True)
        self.reader.SetLayerMode(True)
        if not self.reader.Transfer(self.h_doc):
            raise ValueError('failed to open filename: %s'%filename)

        self.doc = self.h_doc.GetObject()

    def InitOpenRAVE(self):
        if self.orenv is None:
            self.orenv=Environment()
            self.orenv.SetViewer('qtcoin')

    def GetRootShape(self):
        """get the top level shape
        """
        h_shape_tool = XCAFDoc_DocumentTool().ShapeTool(self.doc.Main())
        shape_tool = h_shape_tool.GetObject()
        l_LabelShapes = TDF_LabelSequence()
        shape_tool.GetShapes(l_LabelShapes)
        top_label = shape_tool.NewShape() #this is the "root" label for the assembly
        return shape_tool.GetShape(l_LabelShapes.Value(1)),shape_tool

    def GetFlatShapes(self):
        #
        # Get root assembly
        #
        h_shape_tool = XCAFDoc_DocumentTool().ShapeTool(self.doc.Main())
        shape_tool = h_shape_tool.GetObject()
        # get the top level shapes
        l_LabelShapes = TDF_LabelSequence()
        shape_tool.GetShapes(l_LabelShapes)
        top_label = shape_tool.NewShape() #this is the "root" label for the assembly
        
        labels = []
        shapes = []
        locations = []
        for i in range(1,l_LabelShapes.Length()+1):
            tdflabel = l_LabelShapes.Value(i)
            labels.append(Label(TDF_Label=tdflabel))
            shape = shape_tool.GetShape(tdflabel)
            shapes.append(shape)
            locations.append(shape_tool.GetLocation(tdflabel))

        return labels,shapes,locations

    def triangulateshape(self,shape,deflection=0.0008,scale=1000.0):
        """
        :param deflection: Deflection for tessellation of geometrical lines/surfaces. The minimum distance between two points. Existing mesh is used if its deflection is smaller than the one given by this parameter. Default assumes shape is in mm.
        :param scale: scale that mesh is assumed to be in with respect to meters
        """
        BRepMesh_Mesh(shape,deflection*scale)
        ex = TopExp_Explorer(shape,TopAbs_FACE)
        vertices = []
        indices = []
        iscale = 1.0/scale
        while ex.More():
            F = TopoDS_face(ex.Current())
            L = TopLoc_Location()
            facing = (BRep_Tool_Triangulation(F,L)).GetObject()
            tri = facing.Triangles()
            startindex = len(vertices)
            for i in range(1,facing.NbTriangles()+1):
                trian = tri.Value(i)
                index1, index2, index3 = trian.Get()
                indices.append([startindex+index1-1,startindex+index2-1,startindex+index3-1])
            tab = facing.Nodes()
            for i in range(1,facing.NbNodes()+1):
                pt = tab.Value(i)
                pt.Transform(L.Transformation())
                vertices.append([pt.X()*iscale,pt.Y()*iscale,pt.Z()*iscale])
            ex.Next()

        vertices = numpy.array(vertices)
        indices = numpy.array(indices)
        return vertices, indices

    def GetTriangulatedShapes(self):
        rootshape,shape_tool=self.GetRootShape()
        colors = XCAFDoc_DocumentTool().ColorTool(self.doc.Main()).GetObject()
        it=TopoDS_Iterator(rootshape)
        triangulated_links = []
        while it.More():
            shape=it.Value()
            tdf_label=TDF_Label()
            shape_tool.FindShape(shape,tdf_label)
            label=Label(TDF_Label=tdf_label)
            qc=Quantity.Quantity_Color()
            ret = colors.GetColor(tdf_label,XCAFDoc_ColorSurf,qc) # XCAFDoc_ColorCurv, XCAFDoc_ColorGen
            if ret == 0:
                itc=TDF_ChildIterator(tdf_label)
                while itc.More():
                    ret = colors.GetColor(itc.Value(),XCAFDoc_ColorSurf,qc)
                    if ret != 0:
                        break
                    itc.Next()
                if ret == 0:
                    itc=TDF_ChildIterator(tdf_label.Father())
                    while itc.More():
                        ret = colors.GetColor(itc.Value(),XCAFDoc_ColorSurf,qc)
                        if ret != 0:
                            break
                        itc.Next()
                    if ret == 0:
                        print 'no color for this shape!'
            t = self.TriangleShape()
            t.vertices,t.indices=self.triangulateshape(shape)
            transformation=shape.Location().Transformation()
            t.trans = numpy.reshape([transformation.Value(i,j) for i in range(1,4) for j in range(1,5)],(3,4))
            t.trans[0:3,3] *= 0.001 # convert to meters
            t.name = label.name_attr.decode('iso8859')
            t.color = numpy.array([qc.Red(),qc.Green(),qc.Blue()])
            print t.name,len(t.vertices),t.color
            triangulated_links.append(t)
            it.Next()
        return triangulated_links

    def DrawLinks(self,triangulated_links):
        self.InitOpenRAVE()
        colorchoices = numpy.array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        handles = []
        for t in triangulated_links:
            handles.append(self.orenv.drawtrimesh(t.vertices,t.indices,colors=t.color))
        return handles

    def WriteOpenRAVEXML(self,triangulated_links, linkindiceslist,savedir='',jointlimitsdeg=None):
        mkdir_recursive(savedir)
        for ilink,t in enumerate(triangulated_links):
            buffersize = S = 80 + 4 + 50*len(t.indices)
            b = ctypes.create_string_buffer(buffersize)
            for i in range(5):
                b[i] = 'x'
            for i in range(5,80):
                b[i] = ' '
            offset = 80
            struct.pack_into('I',b,offset,len(t.indices))
            offset += 4
            svertex=struct.Struct('fff')
            sattr=struct.Struct('H')
            for triindices in t.indices:
                svertex.pack_into(b,offset,0,0,0)
                offset += svertex.size
                for vindex in triindices:
                    svertex.pack_into(b,offset,*t.vertices[vindex])
                    offset += svertex.size
                sattr.pack_into(b,offset,0)
                offset += sattr.size
            open(os.path.join(savedir,'geom%d.stl'%ilink),'w').write(b.raw)

        kinbodyxml = '<kinbody name="body">\n'
        for ilink,linkindices in enumerate(linkindiceslist):
            geomxml = ''
            for index in linkindices:
                t = triangulated_links[index]
                filename = 'geom%d.stl'%index
                geomxml += '<geom type="trimesh">\n<diffusecolor>%f %f %f</diffusecolor>\n<data>%s</data>\n<render>%s</render>\n</geom>\n'%(t.color[0],t.color[1],t.color[2],filename,filename)
            kinbodyxml += '<body name="link%d">\n'%ilink + geomxml + '</body>\n'
            if ilink > 0:
                T = triangulated_links[linkindices[0]].trans
                jointxml = '<joint name="j%d" type="hinge">\n'%(ilink-1)
                jointxml += '<body>link%d</body>\n'%(ilink-1)
                jointxml += '<body>link%d</body>\n'%ilink
                if jointlimitsdeg is not None:
                    jointxml += '<limitsdeg>%f %f</limitsdeg>\n'%(jointlimitsdeg[ilink-1][0],jointlimitsdeg[ilink-1][1])
                jointxml += '<anchor>%f %f %f</anchor>\n'%(T[0,3],T[1,3],T[2,3])
                if linkindices[0] == 15:
                    # temp exception
                    jointxml += '<axis>%f %f %f</axis>\n'%(T[0,1],T[1,1],T[2,1])
                else:
                    jointxml += '<axis>%f %f %f</axis>\n'%(T[0,0],T[1,0],T[2,0])
                jointxml += '</joint>\n'
                kinbodyxml += jointxml
        kinbodyxml += '</kinbody>'
        filename = os.path.join(savedir,'body.kinbody.xml')
        open(filename,'w').write(kinbodyxml)
        return filename

    def DrawAxes(self,T,dist=0.1):
        return self.orenv.drawlinelist(numpy.array([T[0:3,3],T[0:3,3]+T[0:3,0]*dist,T[0:3,3],T[0:3,3]+T[0:3,1]*dist,T[0:3,3],T[0:3,3]+T[0:3,2]*dist]),1,colors=numpy.array([[1,0,0],[1,0,0],[0,1,0],[0,1,0],[0,0,1],[0,0,1]]))


if __name__ == "__main__":
    parser = OptionParser()
    usage = "usage: %prog [options] <arg>"
    parser = OptionParser(usage)
    parser.add_option("--linknames", type='string', dest="linknames", action="append", default=None,
                      help='comma separated object names for each link')
    (options, args) = parser.parse_args()
    linknames = [linkname.decode(sys.stdin.encoding) for linkname in options.linknames]
    
    self=RobotImporter(args[0])
    triangulated_links = self.GetTriangulatedShapes()
    handles = self.DrawLinks(triangulated_links)
    raw_input('press any key')
    
def test():
    import importrobotocaf
    filename = '/home/rdiankov/openravetests/URSplitted/Robot_Simpel_r01.STEP'
    self=importrobotocaf.RobotImporter(filename)
    triangulated_links = self.GetTriangulatedShapes()
    handles = self.DrawLinks(triangulated_links)

    linknames = ['Size 3_Round base_r02_Simple,Size 3_House_r11_Simple',
             'Size 3_Joint lid_r06_Simple,Size 3_House_r11_Simple,Size 3_Joint lid_r06_Simple',
             'Size 3_Over arm_r01_Simpel',
             'Size 3_House_r11_Simple,Size 3_Joint lid_r06_Simple',
             'Size 1 rør_r01_Simple,Size 3_Joint lid_r06_Simple',
             'Size 1_Joint lid_r07_Simple,Size 1_House_r13_Simple',
             'Size 1_House_r13_Simple,Size 1_Joint lid_r07_Simple',
             'Size 1_House_r13_Simple,Size 1_Joint lid_r07_Simple,Size 1_Tool flange_r03_Simple']
    linknames = [n.decode('utf-8') for n in linknames]
    trinames = [tri[0] for tri in triangulated_links]
    linkindiceslist = []
    for linkname in linknames:
        names = [n.strip() for n in linkname.split(',')]
        linkindiceslist.append([trinames.index(name) for name in names])
    linkindiceslist = [[2],
                       [10,16],
                       [6,3,8,9,12],
                       [15,4],
                       [5,7],
                       [0,14],
                       [1,11,13]]

    jointlimitsdeg = [[-360.0,360.0]]*7
    geomxml = self.WriteOpenRAVEXML(triangulated_links, linkindiceslist,savedir='test',jointlimitsdeg=jointlimitsdeg)

    trans = [triangulated_links[indices[0]].trans for indices in linkindiceslist]
    handles2 = [self.DrawAxes(T) for T in trans]

    labels,shapes,locations = self.GetFlatShapes()

    vertices,indices=self.triangulateshape(shapes[4])
    h=self.orenv.drawtrimesh(vertices,indices,numpy.array([0,0,1,0.2]))
    h2=self.orenv.plot3(vertices,1,numpy.array([1,0,0,0.2]))            
        
    print "top entry", importrobotocaf.Label(TDF_Label=label).entry

    shape_gid = TDataStd.TDataStd_Shape().ID()

    h_u = TNaming.Handle_TNaming_UsedShapes()
    gid = h_u.GetObject().getid()
    root_label.FindAttribute(gid, h_u)    

    l_Colors = XCAFDoc_DocumentTool().ColorTool(self.doc.Main())
    colors = l_Colors.GetObject()
    l_LabelColors = TDF_LabelSequence()
    colors.GetColors(l_LabelColors)
    for i in range(1,8):
        c = importrobotocaf.Label(TDF_Label=l_LabelColors.Value(i))
        print c.name_attr

    shape_tool.GetShape(l_LabelShapes.Value(1))
    
    
    l_Layers = XCAFDoc_DocumentTool().LayerTool(self.doc.Main())
    layers = l_Layers.GetObject()
    l_LabelLayers = TDF_LabelSequence()
    layers.GetLayerLabels(l_LabelLayers)
    
