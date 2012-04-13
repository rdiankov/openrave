// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_BASEFLASHLIDAR_H
#define OPENRAVE_BASEFLASHLIDAR_H

/// Flash LIDAR - sends laser points given a camera projection matrix
class BaseFlashLidar3DSensor : public SensorBase
{
protected:
    class BaseFlashLidar3DXMLReader : public BaseXMLReader
    {
public:
        BaseFlashLidar3DXMLReader(boost::shared_ptr<BaseFlashLidar3DSensor> psensor) : _psensor(psensor) {
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support ) {
                    return PE_Support;
                }
                return PE_Ignore;
            }
            static boost::array<string, 18> tags = { { "sensor", "minangle", "min_angle", "maxangle", "max_angle", "maxrange", "max_range", "minrange", "min_range", "scantime", "color", "time_scan", "time_increment", "power", "kk", "width", "height"}};
            if( find(tags.begin(),tags.end(),name) == tags.end() ) {
                return PE_Pass;
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
            else if( name == "power" ) {
                ss >> _psensor->_bPower;
            }
            else if((name == "minangle")||(name == "min_angle")) {
                ss >> _psensor->_pgeom->min_angle[0];
                if( !!ss )
                    _psensor->_pgeom->min_angle[0] *= PI/180.0f;                                                                                                                                                                                                      // convert to radians
            }
            else if((name == "maxangle")||(name == "max_angle")) {
                ss >> _psensor->_pgeom->max_angle[0];
                if( !!ss )
                    _psensor->_pgeom->max_angle[0] *= PI/180.0f;                                                                                                                                                                                                      // convert to radians
            }
            else if((name == "maxrange")||(name == "max_range")) {
                ss >> _psensor->_pgeom->max_range;
            }
            else if((name == "scantime")||(name == "time_scan")) {
                ss >> _psensor->_pgeom->time_scan;
            }
            else if( name == "color" ) {
                ss >> _psensor->_vColor.x >> _psensor->_vColor.y >> _psensor->_vColor.z;
                // ok if not everything specified
                if( !ss ) {
                    ss.clear();
                }
            }
            else if( name == "kk" ) {
                ss >> _psensor->_pgeom->KK.fx >> _psensor->_pgeom->KK.fy >> _psensor->_pgeom->KK.cx >> _psensor->_pgeom->KK.cy;
            }
            else if( name == "width" ) {
                ss >> _psensor->_pgeom->width;
            }
            else if( name == "height" ) {
                ss >> _psensor->_pgeom->height;
            }
            else {
                RAVELOG_WARN(str(boost::format("bad tag: %s")%name));
            }
            if( !ss ) {
                RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
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
        boost::shared_ptr<BaseFlashLidar3DSensor> _psensor;
        stringstream ss;
    };

    class BaseFlashLidar3DGeom : public LaserGeomData
    {
public:
        CameraIntrinsics KK;         // intrinsic matrix expanding to [ KK[0] 0 KK[2]; 0 KK[1] KK[3] ]
        int width, height;         // dimensions in number of lasers
    };

public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        return BaseXMLReaderPtr(new BaseFlashLidar3DXMLReader(boost::dynamic_pointer_cast<BaseFlashLidar3DSensor>(ptr)));
    }

    BaseFlashLidar3DSensor(EnvironmentBasePtr penv) : SensorBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nProvides a simulated 3D flash lidar sensor. A flash LIDAR instantaneously returns the depth measurements in the form of an image. It has the same projection parameters as a camera except each pixel is an active element that measures distance. The XML parameters are the same as :ref:`sensor-baselaser2d` along with:\n\
* KK - 4 element vector that constructs the intrinsic matrix of the flash lidar (KK[0] 0 KK[2]; 0 KK[1] KK[3]; 0 0 1]. \n\
* width - the number of active elements along the x-axis.\n\
* height - the number of active elements along the y-axis.\n\
\n\
.. image:: ../../../images/interface_baseflashlidar.jpg\n\
  :width: 400\n\
";
        RegisterCommand("render",boost::bind(&BaseFlashLidar3DSensor::_Render,this,_1,_2),
                        "Set rendering of the plots (1 or 0).");
        RegisterCommand("collidingbodies",boost::bind(&BaseFlashLidar3DSensor::_CollidingBodies,this,_1,_2),
                        "Returns the ids of the bodies that the laser beams have hit.");

        _pgeom.reset(new BaseFlashLidar3DGeom());
        _pdata.reset(new LaserSensorData());
        _report.reset(new CollisionReport());

        _bRenderData = false;
        _bRenderGeometry = true;
        _bPower = false;
        _pgeom->min_angle[0] = -PI/2; _pgeom->min_angle[1] = 0;
        _pgeom->max_angle[0] = PI/2; _pgeom->max_angle[1] = 0;
        _pgeom->max_range = 100;
        _pgeom->KK.fx = 500; _pgeom->KK.fy = 500; _pgeom->KK.cx = 250; _pgeom->KK.cy = 250;
        _pgeom->width = 64; _pgeom->height = 64;
        _fTimeToScan = 0;
        _vColor = RaveVector<float>(0.5f,0.5f,1,1);
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
        case CC_RenderDataOn:
            _bRenderData = true;
            return _bRenderData;
        case CC_RenderDataOff: {
            boost::mutex::scoped_lock lock(_mutexdata);
            _listGraphicsHandles.clear();
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

    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        _RenderGeometry();
        _fTimeToScan -= fTimeElapsed;
        if(( _fTimeToScan <= 0) && _bPower ) {
            _fTimeToScan = _pgeom->time_scan;

            RAY r;

            GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Distance);
            Transform t;

            {
                // Lock the data mutex and fill with the range data (get all in one timestep)
                boost::mutex::scoped_lock lock(_mutexdata);
                t = GetTransform();
                _pdata->__trans = t;
                _pdata->__stamp = GetEnv()->GetSimulationTime();

                r.pos = t.trans;
                _pdata->positions.at(0) = t.trans;

                for(int w = 0; w < _pgeom->width; ++w) {
                    for(int h = 0; h < _pgeom->height; ++h) {
                        Vector vdir;
                        vdir.x = (float)w*_iKK[0] + _iKK[2];
                        vdir.y = (float)h*_iKK[1] + _iKK[3];
                        vdir.z = 1.0f;
                        vdir = t.rotate(vdir.normalize3());
                        r.dir = _pgeom->max_range*vdir;

                        int index = w*_pgeom->height+h;

                        if( GetEnv()->CheckCollision(r, _report)) {
                            _pdata->ranges[index] = vdir*_report->minDistance;
                            _pdata->intensity[index] = 1;
                            // store the colliding bodies
                            KinBody::LinkConstPtr plink = !!_report->plink1 ? _report->plink1 : _report->plink2;
                            if( !!plink ) {
                                _databodyids[index] = plink->GetParent()->GetEnvironmentId();
                            }
                        }
                        else {
                            _databodyids[index] = 0;
                            _pdata->ranges[index] = vdir*_pgeom->max_range;
                            _pdata->intensity[index] = 0;
                        }
                    }
                }

                _report->Reset();
            }

            GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);

            if( _bRenderData ) {
                // If can render, check if some time passed before last update
                list<GraphHandlePtr> listhandles;
                int N = 0;
                vector<RaveVector<float> > vpoints;
                vector<int> vindices;

                {
                    // Lock the data mutex and fill the arrays used for rendering
                    boost::mutex::scoped_lock lock(_mutexdata);
                    N = (int)_pdata->ranges.size();
                    vpoints.resize(N+1);
                    for(int i = 0; i < N; ++i)
                        vpoints[i] = _pdata->ranges[i] + t.trans;
                    vpoints[N] = t.trans;
                }

                // render the transparent fan for every column
                vindices.resize(3*(_pgeom->height-1)*_pgeom->width);
                int index = 0;
                for(int w = 0; w < _pgeom->width; ++w) {
                    for(int i = 0; i < _pgeom->height-1; ++i) {
                        vindices[index++] = w*_pgeom->height+i;
                        vindices[index++] = w*_pgeom->height+i+1;
                        vindices[index++] = N;
                    }
                }

                _vColor.w = 1;
                // Render points at each measurement, and a triangle fan for the entire free surface of the laser
                listhandles.push_back(GetEnv()->plot3(&vpoints[0].x, N, sizeof(vpoints[0]), 5.0f, _vColor));

                _vColor.w = 0.01f;
                listhandles.push_back(GetEnv()->drawtrimesh(&vpoints[0].x, sizeof(vpoints[0]), &vindices[0], vindices.size()/3, _vColor));

                _listGraphicsHandles.swap(listhandles);

            }
            else {
                // destroy graphs
                _listGraphicsHandles.clear();
            }
        }

        return true;
    }

    virtual SensorGeometryPtr GetSensorGeometry(SensorType type)
    {
        if(( type == ST_Invalid) ||( type == ST_Laser) ) {
            BaseFlashLidar3DGeom* pgeom = new BaseFlashLidar3DGeom();
            *pgeom = *_pgeom;
            return SensorGeometryPtr(pgeom);
        }
        return SensorGeometryPtr();
    }

    virtual SensorDataPtr CreateSensorData(SensorType type)
    {
        if(( type == ST_Invalid) ||( type == ST_Laser) ) {
            return SensorDataPtr(new LaserSensorData());
        }
        return SensorDataPtr();
    }

    virtual bool GetSensorData(SensorDataPtr psensordata)
    {
        if( psensordata->GetType() == ST_Laser ) {
            boost::mutex::scoped_lock lock(_mutexdata);
            *boost::dynamic_pointer_cast<LaserSensorData>(psensordata) = *_pdata;
            return true;
        }
        return false;
    }

    virtual bool Supports(SensorType type) {
        return type == ST_Laser;
    }

    bool _Render(ostream& sout, istream& sinput)
    {
        sinput >> _bRenderData;
        return !!sinput;
    }
    bool _CollidingBodies(ostream& sout, istream& sinput)
    {
        boost::mutex::scoped_lock lock(_mutexdata);
        FOREACH(it, _databodyids) {
            sout << *it << " ";
        }
        return true;
    }

    virtual void SetTransform(const Transform& trans)
    {
        _trans = trans;
    }

    virtual Transform GetTransform() {
        return _trans;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        SensorBase::Clone(preference,cloningoptions);
        boost::shared_ptr<BaseFlashLidar3DSensor const> r = boost::dynamic_pointer_cast<BaseFlashLidar3DSensor const>(preference);
        *_pgeom = *r->_pgeom;
        _vColor = r->_vColor;
        std::copy(&r->_iKK[0],&r->_iKK[4],&_iKK[0]);
        _trans = r->_trans;
        _fTimeToScan = r->_fTimeToScan;
        _bRenderGeometry = r->_bRenderGeometry;
        _bRenderData = r->_bRenderData;
        _bPower = r->_bPower;
        _Reset();
    }

protected:
    virtual void _Reset()
    {
        _iKK[0] = 1.0f / _pgeom->KK.fx;
        _iKK[1] = 1.0f / _pgeom->KK.fy;
        _iKK[2] = -_pgeom->KK.cx / _pgeom->KK.fx;
        _iKK[3] = -_pgeom->KK.cy / _pgeom->KK.fy;
        _listGraphicsHandles.clear();
        _pdata->positions.resize(1);
        _pdata->ranges.resize(_pgeom->width*_pgeom->height);
        _pdata->intensity.resize(_pgeom->width*_pgeom->height);
        _databodyids.resize(_pgeom->width*_pgeom->height);
        FOREACH(it, _pdata->ranges) {
            *it = Vector(0,0,0);
        }
        FOREACH(it, _pdata->intensity) {
            *it = 0;
        }
    }

    void _RenderGeometry()
    {
        if( !_bRenderGeometry ) {
            return;
        }
        Transform t = GetTransform();
        if( !_graphgeometry ) {
            vector<RaveVector<float> > viconpoints(5);
            vector<int> viconindices(6*3);
            viconpoints[0] = Vector(0,0,0);
            viconpoints[1] = 0.1f*Vector(_iKK[2], _iKK[3],1);
            viconpoints[2] = 0.1f*Vector(_iKK[2], (float)_pgeom->height*_iKK[1] + _iKK[3],1);
            viconpoints[3] = 0.1f*Vector((float)_pgeom->width*_iKK[0] + _iKK[2], _iKK[3],1);
            viconpoints[4] = 0.1f*Vector((float)_pgeom->width*_iKK[0] + _iKK[2], (float)_pgeom->height*_iKK[1] + _iKK[3],1);

            viconindices[0] = 0; viconindices[1] = 1; viconindices[2] = 2;
            viconindices[3] = 0; viconindices[4] = 4; viconindices[5] = 2;
            viconindices[6] = 0; viconindices[7] = 3; viconindices[8] = 4;
            viconindices[9] = 0; viconindices[10] = 1; viconindices[11] = 3;
            viconindices[12] = 1; viconindices[13] = 3; viconindices[14] = 2;
            viconindices[15] = 3; viconindices[16] = 4; viconindices[17] = 2;

            RaveVector<float> vcolor = _vColor*0.5f;
            vcolor.w = 0.7f;

            _graphgeometry = GetEnv()->drawtrimesh(&viconpoints[0].x, sizeof(viconpoints[0]), &viconindices[0], 6, vcolor);
        }
        if( !!_graphgeometry ) {
            _graphgeometry->SetTransform(t);
        }
    }

    boost::shared_ptr<BaseFlashLidar3DGeom> _pgeom;
    boost::shared_ptr<LaserSensorData> _pdata;
    vector<int> _databodyids;     ///< if non 0, for each point in _data, specifies the body that was hit
    CollisionReportPtr _report;
    // more geom stuff
    RaveVector<float> _vColor;
    dReal _iKK[4];     // inverse of KK

    Transform _trans;
    list<GraphHandlePtr> _listGraphicsHandles;
    GraphHandlePtr _graphgeometry;
    dReal _fTimeToScan;

    boost::mutex _mutexdata;
    bool _bRenderData, _bRenderGeometry, _bPower;

    friend class BaseFlashLidar3DXMLReader;
};

#endif
