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

from enthought.traits.api import (HasTraits, Property, Bool,  on_trait_change, cached_property, Instance, File, Float as _Float, List, Str, Enum, Int)

from openravepy import *
import numpy

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
        for i in range(1,l_LabelShapes.Length()):
            tdflabel = l_LabelShapes.Value(i)
            labels.append(Label(TDF_Label=tdflabel))
            shape = shape_tool.GetShape(tdflabel)
            shapes.append(shape)
            locations.append(shape_tool.GetLocation(tdflabel))
            print locations[-1].ShallowDumpToString()

        return labels,shapes,locations
    
        #l_Colors = XCAFDoc_DocumentTool().ColorTool(self.doc.Main())

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
        handles = []
        it=TopoDS_Iterator(rootshape)
        triangulated_links = []
        while it.More():
            shape=it.Value()
            tdf_label=TDF_Label()
            shape_tool.FindShape(shape,tdf_label)
            label=Label(TDF_Label=tdf_label)
            vertices,indices=self.triangulateshape(shape)
            t=shape.Location().Transformation()
            trans = numpy.reshape([t.Value(i,j) for i in range(1,4) for j in range(1,5)],(3,4))
            print label.name_attr,len(vertices)
            triangulated_links.append([label.name_attr,trans,vertices,indices])
            it.Next()
        return triangulated_links

    def DrawLinks(self,triangulated_links):
        self.InitOpenRAVE()
        handles = []
        for name,trans,vertices,indices in triangulated_links:
            handles.append(self.orenv.drawtrimesh(vertices,indices,numpy.array([0,0,1,0.2])))
        return handles
    
if __name__ == "__main__":
    parser = OptionParser()
    usage = "usage: %prog [options] <arg>"
    parser = OptionParser(usage)
    parser.add_option("--link", type='string', dest="links", action="append", default=None,
                      help='comma separated object names for each link')
    (options, args) = parser.parse_args()
    self=RobotImporter(args[0])
    triangulated_links = self.GetTriangulatedShapes()
    handles = self.DrawLinks(triangulated_links)
    raw_input('press any key')
    
def test():
    import importrobotstep
    filename = '/home/rdiankov/openravetests/URSplitted/Robot_Simpel_r01.STEP'
    self=importrobotstep.RobotImporter(filename)
    triangulated_links = self.GetTriangulatedShapes()
    handles = self.DrawLinks(triangulated_links)
    
    labels,shapes,locations = self.getshapes()

    vertices,indices=self.triangulateshape(shapes[4])
    h=self.orenv.drawtrimesh(vertices,indices,numpy.array([0,0,1,0.2]))
    h2=self.orenv.plot3(vertices,1,numpy.array([1,0,0,0.2]))
            
        
    print "top entry", importrobotstep.Label(TDF_Label=label).entry

    shape_gid = TDataStd.TDataStd_Shape().ID()

    h_u = TNaming.Handle_TNaming_UsedShapes()
    gid = h_u.GetObject().getid()
    root_label.FindAttribute(gid, h_u)
        
