// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
#ifndef OPENRAVE_BASECAMERA_H
#define OPENRAVE_BASECAMERA_H

#include <boost/lexical_cast.hpp>

class BaseCameraSensor : public SensorBase
{
protected:
    class BaseCameraXMLReader : public BaseXMLReader
    {
public:
        BaseCameraXMLReader(boost::shared_ptr<BaseCameraSensor> psensor) : _psensor(psensor) {
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support ) {
                    return PE_Support;
                }
                return PE_Ignore;
            }
            static boost::array<string, 18> tags = { { "sensor", "kk", "width", "height", "framerate", "power", "color", "focal_length","image_dimensions","intrinsic","measurement_time", "format", "distortion_model", "distortion_coeffs", "sensor_reference", "target_region", "gain", "hardware_id"}};
            if( find(tags.begin(),tags.end(),name) == tags.end() ) {
                return PE_Pass;
            }
            if( name == std::string("sensor_reference") ) {
                // read the URL attribute
                FOREACHC(itatt, atts) {
                    if( itatt->first == "url" ) {
                        _psensor->_pgeom->sensor_reference = itatt->second;
//                        size_t startindex = 0;
//                        if( itatt->second.size() > 0 && itatt->second[0] == '#' ) {
//                            startindex = 1;
//                        }
//                        _psensor->_pgeom->sensor_reference = itatt->second.substr(startindex);
                    }
                }
            }
            else if( name == std::string("target_region") ) {
                // read the URL attribute
                FOREACHC(itatt, atts) {
                    if( itatt->first == "url" ) {
                        _psensor->_pgeom->target_region = itatt->second;
//                        size_t startindex = 0;
//                        if( itatt->second.size() > 0 && itatt->second[0] == '#' ) {
//                            startindex = 1;
//                        }
//                        _psensor->_pgeom->target_region = itatt->second.substr(startindex);
                    }
                }
            }
            ss.str("");
            return PE_Support;
        }

        virtual bool endElement(const std::string& name)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(name) ) {
                    _pcurreader.reset();
                }
                return false;
            }
            else if( name == "sensor" ) {
                return true;
            }
            else if((name == "kk")||(name == "KK")) {
                ss >> _psensor->_pgeom->KK.fx >> _psensor->_pgeom->KK.fy >> _psensor->_pgeom->KK.cx >> _psensor->_pgeom->KK.cy;
            }
            else if( name == "intrinsic" ) {
                dReal dummy0, dummy1;
                ss >> _psensor->_pgeom->KK.fx >> dummy0 >> _psensor->_pgeom->KK.cx >> dummy1 >> _psensor->_pgeom->KK.fy >> _psensor->_pgeom->KK.cy;
            }
            else if( name == "focal_length" ) {
                ss >> _psensor->_pgeom->KK.focal_length;
            }
            else if( name == "distortion_model" ) {
                ss >> _psensor->_pgeom->KK.distortion_model;
            }
            else if( name == "distortion_coeffs" ) {
                _psensor->_pgeom->KK.distortion_coeffs = std::vector<dReal>((istream_iterator<dReal>(ss)), istream_iterator<dReal>());
                ss.clear(); // should clear the error since distortion_coeffs read to the end
            }
            else if( name == "image_dimensions" ) {
                // new image_dimensions do not have channels since the number of channels a camera can output is not part of its geometry
                ss >> _psensor->_pgeom->width >> _psensor->_pgeom->height;// >> _psensor->_numchannels;
            }
            else if( name == "width" ) {
                ss >> _psensor->_pgeom->width;
            }
            else if( name == "height" ) {
                ss >> _psensor->_pgeom->height;
            }
            else if( name == "sensor_reference" ) {
                // nothing to do here
            }
            else if( name == "measurement_time" ) {
                ss >> _psensor->_pgeom->measurement_time;
                _psensor->framerate = 1/_psensor->_pgeom->measurement_time;
            }
            else if( name == "framerate" ) {
                ss >> _psensor->framerate;
            }
            else if( name == "target_region" ) {
                // nothing to do here
            }
            else if( name == "gain" ) {
                ss >> _psensor->_pgeom->gain;
            }
            else if( name == "power" ) {
                ss >> _psensor->_bPower;
            }
            else if( name == "format" ) {
                ss >> _psensor->_channelformat;
            }
            else if( name == "color" ) {
                ss >> _psensor->_vColor.x >> _psensor->_vColor.y >> _psensor->_vColor.z;
                // ok if not everything specified
                if( !ss ) {
                    ss.clear();
                }
            }
            else if( name == "hardware_id" ) {
                ss >> _psensor->_pgeom->hardware_id;
            }
            else {
                RAVELOG_WARN(str(boost::format("bad tag: %s")%name));
            }
            if( !ss ) {
                RAVELOG_WARN(str(boost::format("BaseCameraSensor error parsing %s\n")%name));
            }
            return false;
        }

        virtual void characters(const std::string& ch)
        {
            if( !!_pcurreader ) {
                _pcurreader->characters(ch);
            }
            else {
                ss.clear();
                ss << ch;
            }
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        boost::shared_ptr<BaseCameraSensor> _psensor;
        stringstream ss;
    };


    class BaseCameraJSONReader : public BaseJSONReader
    {
public:
        BaseCameraJSONReader() { 
            _pgeom.reset(new CameraGeomData());
        }
        virtual ~BaseCameraJSONReader() {}
        ReadablePtr GetReadable() override {
            return _pgeom;
        }
protected:
        CameraGeomDataPtr _pgeom;
    };

public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        return BaseXMLReaderPtr(new BaseCameraXMLReader(boost::dynamic_pointer_cast<BaseCameraSensor>(ptr)));
    }

    static BaseJSONReaderPtr CreateJSONReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        return BaseJSONReaderPtr(new BaseCameraJSONReader());
    }

    BaseCameraSensor(EnvironmentBasePtr penv) : SensorBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nProvides a simulated camera using the standard pinhole projection.";
        RegisterCommand("power",boost::bind(&BaseCameraSensor::_Power,this,_1,_2), "deprecated");
        RegisterCommand("render",boost::bind(&BaseCameraSensor::_Render,this,_1,_2),"deprecated");
        RegisterCommand("setintrinsic",boost::bind(&BaseCameraSensor::_SetIntrinsic,this,_1,_2),
                        "Set the intrinsic parameters of the camera (fx,fy,cx,cy).");
        RegisterCommand("setdims",boost::bind(&BaseCameraSensor::_SetDims,this,_1,_2),
                        "Set the dimensions of the image (width,height)");
        RegisterCommand("SaveImage",boost::bind(&BaseCameraSensor::_SaveImage,this,_1,_2),
                        "Saves the next camera image to the given filename");
        _pgeom.reset(new CameraGeomData());
        _pdata.reset(new CameraSensorData());
        _bPower = false;
        _vColor = RaveVector<float>(0.5f,0.5f,1,1);
        framerate = 5;
        //_numchannels = 3;
        _bRenderGeometry = true;
        _bRenderData = false;
        _Reset();
    }

    virtual int Configure(ConfigureCommand command, bool blocking)
    {
        switch(command) {
        case CC_PowerOn:
            _bPower = true;
            _Reset();
            return _bPower;
        case CC_PowerOff:
            _bPower = false;
            _Reset();
            return _bPower;
        case CC_PowerCheck:
            return _bPower;
        case CC_RenderDataOn: {
            try {
                if( !_bRenderData ) {
                    stringstream ss;
                    ss << "qtcameraviewer " << GetName();
                    _dataviewer = RaveCreateViewer(GetEnv(),ss.str());
                    _bRenderData = !!_dataviewer;
                    if( _bRenderData ) {
                        _dataviewer->main();
                    }
                }
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN(str(boost::format("BaseCameraSensor::Configure: %s")%ex.what()));
            }
            return _bRenderData;
        }
        case CC_RenderDataOff: {
            boost::mutex::scoped_lock lock(_mutexdata);
            _dataviewer.reset();
            _bRenderData = false;
            return _bRenderData;
        }
        case CC_RenderDataCheck:
            return _bRenderData;
        case CC_RenderGeometryOn:
            _bRenderGeometry = true;
            _RenderGeometry();
            return _bRenderData;
        case CC_RenderGeometryOff: {
            boost::mutex::scoped_lock lock(_mutexdata);
            _graphgeometry.reset();
            _bRenderGeometry = false;
            return _bRenderData;
        }
        case CC_RenderGeometryCheck:
            return _bRenderGeometry;
        }
        throw openrave_exception(str(boost::format("SensorBase::Configure: unknown command 0x%x")%command));
    }

    virtual void _Reset()
    {
        _pdata->vimagedata.resize(0);
        _pdata->__stamp = 0;
        _vimagedata.clear(); // do not resize vector here since it might never be used and it will take up lots of memory!
        _fTimeToImage = 0;
        _graphgeometry.reset();
        _dataviewer.reset();
        _psensor_reference.reset();
        
        boost::shared_ptr<CameraGeomData> pgeom = _pgeom;
        if( !!pgeom && pgeom->sensor_reference.size() > 0 ) {
            // does not exist yet
            SensorBasePtr psensor_reference = GetEnv()->GetSensor(pgeom->sensor_reference);
            if( !psensor_reference ) {
                RAVELOG_VERBOSE_FORMAT("could not find sensor reference %s of sensor %s", pgeom->sensor_reference%_name);
            }
//            else {
//                _psensor_reference = psensor_reference;
//            }
        }
    }

    virtual void SetSensorGeometry(SensorGeometryConstPtr pgeometry)
    {
        OPENRAVE_ASSERT_OP(pgeometry->GetType(), ==, ST_Camera );
        *_pgeom = *boost::static_pointer_cast<CameraGeomData const>(pgeometry);
        _Reset();
    }

    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        boost::shared_ptr<CameraSensorData> pdata = _pdata;

        _RenderGeometry();
        if(( _pgeom->width > 0) &&( _pgeom->height > 0) && _bPower) {
            _fTimeToImage -= fTimeElapsed;
            if( _fTimeToImage <= 0 ) {
                _fTimeToImage = 1 / (float)framerate;
                GetEnv()->UpdatePublishedBodies();
                if( !!GetEnv()->GetViewer() ) {
                    _vimagedata.resize(3*_pgeom->width*_pgeom->height);
                    if( GetEnv()->GetViewer()->GetCameraImage(_vimagedata, _pgeom->width, _pgeom->height, _trans, _pgeom->KK) ) {
                        // copy the data
                        boost::mutex::scoped_lock lock(_mutexdata);
                        pdata->vimagedata = _vimagedata;
                        pdata->__stamp = GetEnv()->GetSimulationTime();
                        pdata->__trans = _trans;
                    }
                }
            }
        }
        return true;
    }

    virtual SensorGeometryConstPtr GetSensorGeometry(SensorType type)
    {
        if(( type == ST_Invalid) ||( type == ST_Camera) ) {
            if( !!_pgeom ) {
                SensorBasePtr psensor_reference = _psensor_reference.lock();
                if( !!psensor_reference ) {
                    _pgeom->sensor_reference = psensor_reference->GetName();
                }
            }
            CameraGeomData* pgeom = new CameraGeomData();
            *pgeom = *_pgeom;
            return SensorGeometryConstPtr(boost::shared_ptr<CameraGeomData>(pgeom));
        }
        return SensorGeometryConstPtr();
    }

    virtual SensorDataPtr CreateSensorData(SensorType type)
    {
        if(( type == ST_Invalid) ||( type == ST_Camera) ) {
            return SensorDataPtr(boost::shared_ptr<CameraSensorData>(new CameraSensorData()));
        }
        return SensorDataPtr();
    }

    virtual bool GetSensorData(SensorDataPtr psensordata)
    {
        if( _bPower &&( psensordata->GetType() == ST_Camera) ) {
            boost::mutex::scoped_lock lock(_mutexdata);
            if( _pdata->vimagedata.size() > 0 ) {
                *boost::dynamic_pointer_cast<CameraSensorData>(psensordata) = *_pdata;
                return true;
            }
        }
        return false;
    }

    virtual bool Supports(SensorType type) {
        return type == ST_Camera;
    }

    bool _Power(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("power command deprecated, use SensorBase::Configure(CC_PowerOn)\n");
        sinput >> _bPower;
        if( !_bPower ) {
            // should reset!
            _pdata->vimagedata.resize(0);
            _pdata->__stamp = 0;
        }
        return !!sinput;
    }
    bool _Render(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("Render command deprecated, use SensorBase::Configure(CC_RenderGeometryOn)\n");
        sinput >> _bRenderGeometry;
        return !!sinput;
    }
    bool _SetIntrinsic(ostream& sout, istream& sinput)
    {
        sinput >> _pgeom->KK.fx >> _pgeom->KK.fy >> _pgeom->KK.cx >> _pgeom->KK.cy;
        return !!sinput;
    }
    bool _SetDims(ostream& sout, istream& sinput)
    {
        sinput >> _pgeom->width >> _pgeom->height;
        if( !!sinput ) {
            _Reset();
            return true;
        }
        return false;
    }
    bool _SaveImage(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("SaveImage not implemented yet\n");
        return false;
    }

    virtual void SetTransform(const Transform& trans)
    {
        _trans = trans;
    }

    virtual const Transform& GetTransform() {
        return _trans;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        SensorBase::Clone(preference,cloningoptions);
        boost::shared_ptr<BaseCameraSensor const> r = boost::dynamic_pointer_cast<BaseCameraSensor const>(preference);
        // r->_pgeom->sensor_reference should already be correct
        *_pgeom = *r->_pgeom;
        _vColor = r->_vColor;
        _trans = r->_trans;
        _fTimeToImage = r->_fTimeToImage;
        framerate = r->framerate;
        _bRenderGeometry = r->_bRenderGeometry;
        _bRenderData = r->_bRenderData;
        _bPower = r->_bPower;
        _psensor_reference.reset();
        _Reset();
    }

    void Serialize(BaseXMLWriterPtr writer, int options=0) const override
    {
        if( !!_pgeom ) {
            SensorBasePtr psensor_reference = _psensor_reference.lock();
            if( !!psensor_reference ) {
                _pgeom->sensor_reference = psensor_reference->GetName();
            }
        }
        _pgeom->SerializeXML(writer, options);
        AttributesList atts;
        stringstream ss;
        ss << _vColor.x << " " << _vColor.y << " " << _vColor.z;
        writer->AddChild("color",atts)->SetCharData(ss.str());
        writer->AddChild("format",atts)->SetCharData(_channelformat.size() > 0 ? _channelformat : std::string("uint8"));
    }

    virtual void SetName(const std::string& newname)
    {
        boost::shared_ptr<CameraGeomData> pgeom = _pgeom;
        if( !!pgeom && pgeom->sensor_reference.size() > 0 ) {
            // does not exist yet
            SensorBasePtr psensor_reference = _psensor_reference.lock();
            if( !psensor_reference ) {
                // HACK for collada: check if pgeom->sensor_reference is in robotname:sensorname and that matches with the current sensor name's robot. If it does, then most likely robot is changing its name and sensor_reference is part of the same changing robot, so have to update sensor_reference! The only way to resolve this issue is to introduce unique IDs in openrave, slated for openrave JSON format.
                if( newname.find(':') != std::string::npos && _name.find(':') != std::string::npos && pgeom->sensor_reference.find(':') != std::string::npos) {
                    std::string oldrobotname = _name.substr(0, _name.find(':'));
                    std::string newrobotname = newname.substr(0, newname.find(':'));
                    std::string oldreferencerobotname = pgeom->sensor_reference.substr(0, pgeom->sensor_reference.find(':'));
                    
                    if( oldrobotname == oldreferencerobotname ) {
                        std::string newreference = newrobotname + ":" + pgeom->sensor_reference.substr(pgeom->sensor_reference.find(':')+1);
                        RAVELOG_DEBUG_FORMAT("old reference name %s matches with old sensor %s, so changing name to %s", oldrobotname%pgeom->sensor_reference%newreference);
                        pgeom->sensor_reference = newreference;
                        SensorBasePtr psensor_reference = GetEnv()->GetSensor(pgeom->sensor_reference);
                        if( !!psensor_reference ) {
                            _psensor_reference = psensor_reference;
                        }
                    }
                }
                //RAVELOG_WARN_FORMAT("could not find sensor reference %s of sensor %s", pgeom->sensor_reference%_name);
            }
        }

        SensorBase::SetName(newname);
    }

protected:
    void _RenderGeometry()
    {
        if( !_bRenderGeometry ) {
            return;
        }
        if( !_graphgeometry ) {
            // render a simple frustum outlining camera's dimension
            // the frustum is colored with vColor, the x and y axes are colored separetely
            Vector points[7];
            dReal ik0 = 1/_pgeom->KK.fx, ik1 = 1/_pgeom->KK.fy;
            points[0] = Vector(0,0,0);
            points[1] = Vector(((dReal)_pgeom->width-_pgeom->KK.cx)*ik0, ((dReal)_pgeom->height-_pgeom->KK.cy)*ik1, 1);
            points[2] = Vector(-_pgeom->KK.cx*ik0, ((dReal)_pgeom->height-_pgeom->KK.cy)*ik1, 1);
            points[3] = Vector(-_pgeom->KK.cx*ik0, -_pgeom->KK.cy*ik1, 1);
            points[4] = Vector(((dReal)_pgeom->width-_pgeom->KK.cx)*ik0, -_pgeom->KK.cy*ik1, 1);
            points[5] = Vector(0.5f,0,0);
            points[6] = Vector(0,0.5f,0);

            boost::array<int,16> inds = { { 0,1,2,3,4,1,4,0,2,3,0,0,5,0,0,6}};
            vector<RaveVector<float> > viconpoints(inds.size());
            vector<float> vcolors(inds.size()*3);

            for(size_t i = 0; i < inds.size(); ++i) {
                viconpoints[i] = _pgeom->KK.focal_length*points[inds[i]];
                vcolors[3*i+0] = _vColor.x;
                vcolors[3*i+1] = _vColor.y;
                vcolors[3*i+2] = _vColor.z;
            }

            float xaxis[3] = { 1,0,0};
            float yaxis[3] = { 0,1,0};
            float* pstart = &vcolors[vcolors.size()-3*5];
            for(int i = 0; i < 3; ++i) {
                pstart[i] = pstart[3+i] = pstart[6+i] = xaxis[i];
                pstart[9+i] = pstart[12+i] = yaxis[i];
            }
            _graphgeometry = GetEnv()->drawlinestrip(&viconpoints[0].x, viconpoints.size(), sizeof(viconpoints[0]), 1, &vcolors[0]);
        }
        if( !!_graphgeometry ) {
            _graphgeometry->SetTransform(_trans);
        }
    }

    boost::shared_ptr<CameraGeomData> _pgeom;
    boost::shared_ptr<CameraSensorData> _pdata;

    // more geom stuff
    vector<uint8_t> _vimagedata;
    RaveVector<float> _vColor;
    SensorBaseWeakPtr _psensor_reference; ///< weak pointer to the sensor reference. Used to keep track of name changes!

    Transform _trans;
    dReal _fTimeToImage;
    float framerate;
    //int _numchannels;
    GraphHandlePtr _graphgeometry;
    ViewerBasePtr _dataviewer;
    string _channelformat;

    mutable boost::mutex _mutexdata;

    bool _bRenderGeometry, _bRenderData;
    bool _bPower;     ///< if true, gather data, otherwise don't

    friend class BaseCameraXMLReader;
};

#endif
