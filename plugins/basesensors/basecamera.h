// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu), Carnegie Mellon University
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

class BaseCameraSensor : public SensorBase
{
 protected:
    class BaseCameraXMLReader : public BaseXMLReader
    {
    public:
    BaseCameraXMLReader(boost::shared_ptr<BaseCameraSensor> psensor) : _psensor(psensor) {}
        
        virtual ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support )
                    return PE_Support;
                return PE_Ignore;
            }
            
            if( name != "sensor" && name != "kk" && name != "width" && name != "height" && name != "framerate" && name != "power" && name != "color" ) {
                return PE_Pass;
            }
            ss.str("");
            return PE_Support;
        }

        virtual bool endElement(const std::string& name)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(name) )
                    _pcurreader.reset();
                return false;
            }
            else if( name == "sensor" )
                return true;
            else if( name == "kk" )
                ss >> _psensor->_pgeom->KK.fx >> _psensor->_pgeom->KK.fy >> _psensor->_pgeom->KK.cx >> _psensor->_pgeom->KK.cy;
            else if( name == "width" )
                ss >> _psensor->_pgeom->width;
            else if( name == "height" )
                ss >> _psensor->_pgeom->height;
            else if( name == "framerate" )
                ss >> _psensor->framerate;
            else if( name == "power" )
                ss >> _psensor->_bPower;
            else if( name == "color" ) {
                ss >> _psensor->_vColor.x >> _psensor->_vColor.y >> _psensor->_vColor.z;
                // ok if not everything specified
                if( !ss ) {
                    ss.clear();
                }
            }
            else {
                RAVELOG_WARNA(str(boost::format("bad tag: %s")%name));
            }
            if( !ss ) {
                RAVELOG_WARNA(str(boost::format("BaseCameraSensor error parsing %s\n")%name));
            }
            return false;
        }
        
        virtual void characters(const std::string& ch)
        {
            if( !!_pcurreader )
                _pcurreader->characters(ch);
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
 public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const std::list<std::pair<std::string,std::string> >& atts)
    {
        return BaseXMLReaderPtr(new BaseCameraXMLReader(boost::dynamic_pointer_cast<BaseCameraSensor>(ptr)));
    }
    
 BaseCameraSensor(EnvironmentBasePtr penv) : SensorBase(penv) {
        __description = ":Interface Author: Rosen Diankov\nProvides a simulated camera using the standard pinhole projection.";
        RegisterCommand("power",boost::bind(&BaseCameraSensor::_Power,this,_1,_2),
                        "Set the power (1 or 0) of the sensor.");
        RegisterCommand("render",boost::bind(&BaseCameraSensor::_Render,this,_1,_2),
                        "Set rendering of the plots (1 or 0).");
        RegisterCommand("setintrinsic",boost::bind(&BaseCameraSensor::_SetIntrinsic,this,_1,_2),
                        "Set the intrinsic parameters of the camera (fx,fy,cx,cy).");
        RegisterCommand("setdims",boost::bind(&BaseCameraSensor::_SetDims,this,_1,_2),
                    "Set the dimensions of the image (width,height)");
        _pgeom.reset(new CameraGeomData());
        _pdata.reset(new CameraSensorData());
        _bPower = false;
        _vColor = RaveVector<float>(0.5f,0.5f,1,1);
        framerate = 5;
        _bUpdateCameraPlot = true;
    }
    
    ~BaseCameraSensor() {
        Reset(0);
    }
    
    virtual bool Init(const string& args)
    {
        Reset(0);
        return true;
    }

    virtual void Reset(int options)
    {
        _iconhandle.reset();
        _pdata->vimagedata.resize(3*_pgeom->width*_pgeom->height);
        vimagedata.resize(3*_pgeom->width*_pgeom->height);
        fTimeToImage = 0;
        _bUpdateCameraPlot = true;
    }
    
    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        boost::shared_ptr<CameraSensorData> pdata = _pdata;

        if( _bUpdateCameraPlot ) {            
            if( !_iconhandle ) {
                // render a simple frustum outlining camera's dimension
                // the frustum is colored with vColor, the x and y axes are colored separetely
                Vector points[7];
                float ik0 = 1/_pgeom->KK.fx, ik1 = 1/_pgeom->KK.fy;
                points[0] = Vector(0,0,0);
                points[1] = Vector(((float)_pgeom->width-_pgeom->KK.cx)*ik0, ((float)_pgeom->height-_pgeom->KK.cy)*ik1, 1);
                points[2] = Vector(-_pgeom->KK.cx*ik0, ((float)_pgeom->height-_pgeom->KK.cy)*ik1, 1);
                points[3] = Vector(-_pgeom->KK.cx*ik0, -_pgeom->KK.cy*ik1, 1);
                points[4] = Vector(((float)_pgeom->width-_pgeom->KK.cx)*ik0, -_pgeom->KK.cy*ik1, 1);
                points[5] = Vector(0.5f,0,0);
                points[6] = Vector(0,0.5f,0);

                boost::array<int,16> inds = {{0,1,2,3,4,1,4,0,2,3,0,0,5,0,0,6}};

                viconpoints.resize(inds.size());
                vector<float> vcolors(inds.size()*3);

                for(size_t i = 0; i < inds.size(); ++i) {
                    viconpoints[i] = 0.02*points[inds[i]];
                    vcolors[3*i+0] = _vColor.x;
                    vcolors[3*i+1] = _vColor.y;
                    vcolors[3*i+2] = _vColor.z;
                }

                float xaxis[3] = {1,0,0};
                float yaxis[3] = {0,1,0};
                float* pstart = &vcolors[vcolors.size()-3*5];
                for(int i = 0; i < 3; ++i) {
                    pstart[i] = pstart[3+i] = pstart[6+i] = xaxis[i];
                    pstart[9+i] = pstart[12+i] = yaxis[i];
                }
                _iconhandle = GetEnv()->drawlinestrip(viconpoints[0], viconpoints.size(), sizeof(viconpoints[0]), 1, &vcolors[0]);
            }
            if( !!_iconhandle ) {
                _iconhandle->SetTransform(_trans);
            }
        }

        if( _pgeom->width > 0 && _pgeom->height > 0 && _bPower) {
            fTimeToImage -= fTimeElapsed;
            if( fTimeToImage <= 0 ) {
                fTimeToImage = 1 / (float)framerate;
                GetEnv()->UpdatePublishedBodies();
                if( !!GetEnv()->GetViewer() ) {
                    if( GetEnv()->GetViewer()->GetCameraImage(vimagedata, _pgeom->width, _pgeom->height, _trans, _pgeom->KK) ) {
                        // copy the data
                        boost::mutex::scoped_lock lock(_mutexdata);
                        pdata->t = _trans;
                        pdata->vimagedata = vimagedata;
                        pdata->__stamp = GetEnv()->GetSimulationTime();
                    }
                }
            }
        }
        return true;
    }

    virtual SensorGeometryPtr GetSensorGeometry(SensorType type)
    {
        if( type == ST_Invalid || type == ST_Camera ) {
            CameraGeomData* pgeom = new CameraGeomData();
            *pgeom = *_pgeom;
            return SensorGeometryPtr(pgeom);
        }
        return SensorGeometryPtr();
    }

    virtual SensorDataPtr CreateSensorData(SensorType type)
    {
        if( type == ST_Invalid || type == ST_Camera ) {
            return SensorDataPtr(new CameraSensorData());
        }
        return SensorDataPtr();
    }

    virtual bool GetSensorData(SensorDataPtr psensordata)
    {
        if( psensordata->GetType() == ST_Camera ) {
            boost::mutex::scoped_lock lock(_mutexdata);
            *boost::dynamic_pointer_cast<CameraSensorData>(psensordata) = *_pdata;
            return true;
        }
        return false;
    }

    virtual bool Supports(SensorType type) { return type == ST_Camera; }

    bool _Power(ostream& sout, istream& sinput)
    {
        sinput >> _bPower;
        return !!sinput;
    }
    bool _Render(ostream& sout, istream& sinput)
    {
        sinput >> _bUpdateCameraPlot;
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
            Reset(0);
            return true;
        }
        return false;
    }

    virtual void SetTransform(const Transform& trans)
    {
        _trans = trans;
        _bUpdateCameraPlot = true;
    }

    virtual Transform GetTransform() { return _trans; }

    virtual bool Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        boost::shared_ptr<BaseCameraSensor const> r = boost::dynamic_pointer_cast<BaseCameraSensor const>(preference);
        *_pgeom = *r->_pgeom;
        _vColor = r->_vColor;
        _trans = r->_trans;
        fTimeToImage = r->fTimeToImage;
        framerate = r->framerate;
        _bUpdateCameraPlot = r->_bUpdateCameraPlot;
        _bPower = r->_bPower;
        Reset(0);
        return true;
    }

 protected:
    boost::shared_ptr<CameraGeomData> _pgeom;
    boost::shared_ptr<CameraSensorData> _pdata;

    // more geom stuff
    vector<uint8_t> vimagedata;
    RaveVector<float> _vColor;

    Transform _trans;
    dReal fTimeToImage;
    int framerate;
    GraphHandlePtr _iconhandle;
    vector<RaveVector<float> > viconpoints;

    mutable boost::mutex _mutexdata;

    bool _bUpdateCameraPlot;
    bool _bPower; ///< if true, gather data, otherwise don't

    friend class BaseCameraXMLReader;
};

#endif
