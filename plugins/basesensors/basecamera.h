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
        
        virtual void startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
        {
            BaseXMLReader::startElement(name,atts);
            if( !!_pcurreader )
                _pcurreader->startElement(name,atts);
            else if( name != "sensor" && name != "kk" && name != "width" && name != "height" && name != "framerate" && name != "power" && name != "color" ) {
                _pcurreader.reset(new DummyXMLReader(name, "sensor"));
            }
        }

        virtual bool endElement(const std::string& name)
        {
            BaseXMLReader::endElement(name);
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
                if( !ss )
                    ss.clear();
            }
            else
                RAVELOG_WARNA(str(boost::format("bad tag: %s")%name));

            if( !ss )
                RAVELOG_WARNA(str(boost::format("BaseCameraSensor error parsing %s\n")%name));

            return false;
        }
        
        virtual void characters(const std::string& ch)
        {
            BaseXMLReader::characters(ch);
            if( !!_pcurreader )
                _pcurreader->characters(ch);
            else {
                ss.clear();
                ss.str(ch);
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
        _pgeom.reset(new CameraGeomData());
        _pdata.reset(new CameraSensorData());
        _bShowCameraImage = false;
        _bPower = true;
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
        _globid = 0;
        _bUpdateCameraPlot = true;
    }
    
    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        boost::shared_ptr<CameraSensorData> pdata = _pdata;

        if( 0&&_bUpdateCameraPlot ) {
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
                viconpoints[i] = _trans * (0.02*points[inds[i]]);
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

        if( _pgeom->width > 0 && _pgeom->height > 0 && _bPower) {
            fTimeToImage -= fTimeToImage;
            if( fTimeToImage <= 0 ) {
                fTimeToImage = 1 / (float)framerate;
                GetEnv()->UpdatePublishedBodies();
                if( !GetEnv()->GetCameraImage(vimagedata, _pgeom->width, _pgeom->height, _trans, _pgeom->KK) )
                    RAVELOG_ERRORA("camera failed to get image\n");
                //GetEnv()->WriteCameraImage(_pgeom->width, _pgeom->height, _trans, _pgeom->KK,"cam.jpg","jpg");
                
                // copy the data
                boost::mutex::scoped_lock lock(_mutexdata);
                pdata->t = _trans;
                pdata->vimagedata = vimagedata;
                pdata->id = ++_globid;
            }
        }
        
        return true;
    }

    virtual SensorGeometryPtr GetSensorGeometry()
    {
        CameraGeomData* pgeom = new CameraGeomData();
        *pgeom = *_pgeom;
        return SensorGeometryPtr(pgeom);
    }

    virtual SensorDataPtr CreateSensorData()
    {
        return SensorDataPtr(new CameraSensorData());
    }

    virtual bool GetSensorData(SensorDataPtr psensordata)
    {
        boost::mutex::scoped_lock lock(_mutexdata);
        *boost::dynamic_pointer_cast<CameraSensorData>(psensordata) = *_pdata;
        return true;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        if( !is )
            throw openrave_exception("no command",ORE_InvalidArguments);
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "show" )
            is >> _bShowCameraImage;
        else if( cmd == "power" )
            is >> _bPower;
        else if( cmd == "setintrinsic")
            is >> _pgeom->KK.fx >> _pgeom->KK.fy >> _pgeom->KK.cx >> _pgeom->KK.cy;
        else if( cmd == "setdims" ) {
            is >> _pgeom->width >> _pgeom->height;
            Reset(0);
        }
    
        return !!is;
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
        _globid = r->_globid;
        fTimeToImage = r->fTimeToImage;
        framerate = r->framerate;
        _bShowCameraImage = r->_bShowCameraImage;
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
    EnvironmentBase::GraphHandlePtr _iconhandle;
    vector<RaveVector<float> > viconpoints;
    int _globid;

    mutable boost::mutex _mutexdata;

    bool _bShowCameraImage; ///< if true, will show the camera image
    bool _bUpdateCameraPlot;
    bool _bPower; ///< if true, gather data, otherwise don't

    friend class BaseCameraXMLReader;
};

#endif
