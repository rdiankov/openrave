// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#include "plugindefs.h"
#include "baselaser.h"

////////////////////////
/// Regular 2D Laser ///
////////////////////////

void BaseLaser2DSensor::BaseLaser2DXMLReader::startElement(void *ctx, const char *name, const char **atts)
{
}

bool BaseLaser2DSensor::BaseLaser2DXMLReader::endElement(void *ctx, const char *name)
{    
    if( stricmp((const char*)name, "sensor") == 0 ) {
        return true;
    }
    else if( stricmp((const char*)name, "minangle") == 0 ) {
        ss >> _psensor->_geom.min_angle[0];
        if( !!ss )
            _psensor->_geom.min_angle[0] *= PI/180.0f; // convert to radians
    }
    else if( stricmp((const char*)name, "maxangle") == 0 ) {
        ss >> _psensor->_geom.max_angle[0];
        if( !!ss )
            _psensor->_geom.max_angle[0] *= PI/180.0f; // convert to radians
    }
    else if( stricmp((const char*)name, "resolution") == 0 ) {
        ss >> _psensor->_geom.resolution[0];
        if( !!ss )
            _psensor->_geom.resolution[0] *= PI/180.0f; // convert to radians
    }
    else if( stricmp((const char*)name, "maxrange") == 0 ) {
        ss >> _psensor->_geom.max_range;
    }
    else if( stricmp((const char*)name, "scantime") == 0 ) {
        ss >> _psensor->fScanTime;
    }
    else if( stricmp((const char*)name, "color") == 0 ) {
        ss >> _psensor->_vColor.x >> _psensor->_vColor.y >> _psensor->_vColor.z;
        // ok if not everything specified
        if( !ss )
            ss.clear();
    }
    else
        RAVEPRINT(L"unknown field %s\n", name);

    if( !ss )
        RAVEPRINT(L"BaseLaser2DSensor error parsing %s\n", name);

    return false;
}

void BaseLaser2DSensor::BaseLaser2DXMLReader::characters(void *ctx, const char *ch, int len)
{
    if( len > 0 ) {
        ss.clear();
        ss.str(string(ch, len));
    }
    else
        ss.str(""); // reset
}

BaseLaser2DSensor::BaseLaser2DSensor(EnvironmentBase* penv) : SensorBase(penv)
{
    _bRender = false;
    _geom.min_angle[0] = -PI/2; _geom.min_angle[1] = 0;
    _geom.max_angle[0] = PI/2; _geom.max_angle[1] = 0;
    _geom.resolution[0] = 0.01f; _geom.resolution[1] = 0;
    _geom.max_range = 100;
    fTimeToScan = 0;
    _pIconHandle = NULL;
    _vColor = RaveVector<float>(0.5f,0.5f,1,1);
    pthread_mutex_init(&_mutexdata, NULL);
}

BaseLaser2DSensor::~BaseLaser2DSensor()
{
    Reset(0);
    pthread_mutex_destroy(&_mutexdata);
}

bool BaseLaser2DSensor::Init(const char* args)
{
    stringstream ss(args != NULL ? args : "");

    Reset(0);

    return true;
}

void BaseLaser2DSensor::Reset(int options)
{
    FOREACH(it, _listGraphicsHandles)
        GetEnv()->closegraph(*it);
    _listGraphicsHandles.clear();

    if( _pIconHandle != NULL ) {
        GetEnv()->closegraph(_pIconHandle);
        _pIconHandle = NULL;
    }

    int N = (int)( (_geom.max_angle[0]-_geom.min_angle[0])/_geom.resolution[0] + 0.5f)+1;
    
    _data.positions.clear();
    _data.ranges.resize(N);
    _data.intensity.resize(N);
    _databodyids.resize(N);

    FOREACH(it, _data.ranges)
        *it = Vector(0,0,0);
    FOREACH(it, _data.intensity)
        *it = 0;

    _globid = 0;
}

bool BaseLaser2DSensor::SimulationStep(dReal fTimeElapsed)
{
    fTimeToScan -= fTimeElapsed;
    if( fTimeToScan <= 0 ) {
        fTimeToScan = fScanTime;
        Vector rotaxis(0,0,1);
        Transform trot;
        RAY r;
    
        GetEnv()->SetCollisionOptions(CO_Distance);
        COLLISIONREPORT report;    
        Transform t;

        {
            // Lock the data mutex and fill with the range data (get all in one timestep)
            MutexLock m(&_mutexdata);
            _data.t = GetTransform();
            _data.id = ++_globid;
        
            t = GetLaserPlaneTransform();
            r.pos = t.trans;

            size_t index = 0;
            for(float frotangle = _geom.min_angle[0]; frotangle <= _geom.max_angle[0]; frotangle += _geom.resolution[0], ++index) {
                if( index >= _data.ranges.size() )
                    break;
            
                trot.rotfromaxisangle(rotaxis, (dReal)frotangle);
                r.dir = _geom.max_range*t.rotate(trot.rotate(Vector(1,0,0)));
            
                if( GetEnv()->CheckCollision(r, &report)) {
                    _data.ranges[index] = r.dir*report.minDistance;
                    _data.intensity[index] = 1;
                    // store the colliding bodies
                    KinBody::Link* plink = report.plink1 != NULL ? report.plink1 : report.plink2;
                    if( plink != NULL ) {
                        _databodyids[index] = plink->GetParent()->GetNetworkId();
                        //RAVELOG(L"point %d collided with %S:%S\n", index, plink->GetParent()->GetName(), plink->GetName());
                    }
                }
                else {
                    _databodyids[index] = 0;
                    _data.ranges[index] = r.dir*_geom.max_range;
                    _data.intensity[index] = 0;
                }
            }
        }

        GetEnv()->SetCollisionOptions(0);
    
        if( _bRender ) {

            // If can render, check if some time passed before last update
            list<void*> listhandles;
            int N = 0;
            vector<RaveVector<float> > vpoints;
            vector<int> vindices;

            {
                // Lock the data mutex and fill the arrays used for rendering
                MutexLock m(&_mutexdata);
                N = (int)_data.ranges.size();
                vpoints.resize(N+1);
                for(int i = 0; i < N; ++i)
                    vpoints[i] = _data.ranges[i] + t.trans;
                vpoints[N] = t.trans;
            }

            // render the transparent fan
            vindices.resize(3*(N-1));
            
            for(int i = 0; i < N-1; ++i) {
                vindices[3*i+0] = i;
                vindices[3*i+1] = i+1;
                vindices[3*i+2] = N;
            }

            _vColor.w = 1;
            // Render points at each measurement, and a triangle fan for the entire free surface of the laser
            listhandles.push_back(GetEnv()->plot3(&vpoints[0].x, N, sizeof(vpoints[0]), 5.0f, _vColor));
            
            _vColor.w = 0.2f;
            listhandles.push_back(GetEnv()->drawtrimesh(vpoints[0], sizeof(vpoints[0]), &vindices[0], N-1, _vColor));
            
            // close the old graphs last to avoid flickering
            FOREACH(it, _listGraphicsHandles)
                GetEnv()->closegraph(*it);
            _listGraphicsHandles.swap(listhandles);

        }
        else {
            // destroy graphs
            FOREACH(it, _listGraphicsHandles)
                GetEnv()->closegraph(*it);
            _listGraphicsHandles.resize(0);
        }
    }

    return true;
}

SensorBase::SensorGeometry* BaseLaser2DSensor::GetSensorGeometry()
{
    LaserGeomData* pgeom = new LaserGeomData();
    *pgeom = _geom;
    return pgeom;
}

SensorBase::SensorData* BaseLaser2DSensor::CreateSensorData()
{
    return new LaserSensorData();
}

bool BaseLaser2DSensor::GetSensorData(SensorData* psensordata)
{
    if( psensordata == NULL || psensordata->GetType() != ST_Laser )
        return false;

    MutexLock m(&_mutexdata);
    *(LaserSensorData*)psensordata = _data;
    return true;
}

bool BaseLaser2DSensor::SendCmd(std::istream& is, std::ostream& os)
{
    string cmd;
    is >> cmd;

    if( !is )
        return false;

    if( stricmp(cmd.c_str(), "show") == 0 || stricmp(cmd.c_str(), "render") == 0) {
        is >> _bRender;
    }
    else if( stricmp(cmd.c_str(), "collidingbodies") == 0 ) {
        FOREACH(it, _databodyids)
            os << *it << " ";
    }

    return !!is;
}

bool BaseLaser2DSensor::SupportsCmd(const char* pcmd)
{
    if( pcmd == NULL )
        return false;
    return stricmp(pcmd, "show") == 0 || stricmp(pcmd, "render") == 0 || stricmp(pcmd, "collidingbodies") == 0;
}

void BaseLaser2DSensor::SetTransform(const Transform& trans)
{
    _trans = trans;

    // draw new graph
    if( _pIconHandle != NULL ) {
        GetEnv()->closegraph(_pIconHandle);
        _pIconHandle = NULL;
    }

    Transform t = GetLaserPlaneTransform();
    
    int N = 10;
    viconpoints.resize(N+2);
    viconindices.resize(3*N);
    viconpoints[0] = t.trans;
    Transform trot;

    for(int i = 0; i <= N; ++i) {
        dReal fang = _geom.min_angle[0] + (_geom.max_angle[0]-_geom.min_angle[0])*(float)i/(float)N;
        trot.rotfromaxisangle(Vector(0,0,1), fang);
        viconpoints[i+1] = t * trot.rotate(Vector(0.05f,0,0));

        if( i < N ) {
            viconindices[3*i+0] = 0;
            viconindices[3*i+1] = i+1;
            viconindices[3*i+2] = i+2;
        }
    }

    RaveVector<float> vcolor = _vColor*0.5f;
    vcolor.w = 0.7f;
    _pIconHandle = GetEnv()->drawtrimesh(viconpoints[0], sizeof(viconpoints[0]), &viconindices[0], N, vcolor);
}

//////////////////////
/// Spinning Laser ///
//////////////////////
bool BaseSpinningLaser2DSensor::BaseSpinningLaser2DXMLReader::endElement(void *ctx, const char *name)
{
    BaseSpinningLaser2DSensor* psensor = dynamic_cast<BaseSpinningLaser2DSensor*>(_psensor);
    assert( psensor != NULL );
    
    if( stricmp((const char*)name, "spinaxis") == 0 )
        ss >> psensor->_vGeomSpinAxis.x >> psensor->_vGeomSpinAxis.y >> psensor->_vGeomSpinAxis.z;
    else if( stricmp((const char*)name, "spinpos") == 0 )
        ss >> psensor->_vGeomSpinPos.x >> psensor->_vGeomSpinPos.y >> psensor->_vGeomSpinPos.z;
    else if( stricmp((const char*)name, "spinspeed") == 0 ) {
        ss >> psensor->_fGeomSpinSpeed;
    }
    else
        return BaseLaser2DXMLReader::endElement(ctx, name);

    if( !ss )
        RAVEPRINT(L"BaseSpinningLaser2DSensor error parsing xml\n");

    return false;
}

BaseSpinningLaser2DSensor::BaseSpinningLaser2DSensor(EnvironmentBase* penv) : BaseLaser2DSensor(penv)
{
    _fGeomSpinSpeed = 0;
    _vGeomSpinAxis = Vector(1,0,0);
    _fCurAngle = 0;
    _bSpinning = true;
}
    
bool BaseSpinningLaser2DSensor::Init(const char* args)
{
    return BaseLaser2DSensor::Init(args);
}

void BaseSpinningLaser2DSensor::Reset(int options)
{
    BaseLaser2DSensor::Reset(options);
    _fCurAngle = 0;
    _bSpinning = true;
}

bool BaseSpinningLaser2DSensor::SimulationStep(dReal fTimeElapsed)
{
    if( _bSpinning ) {
        _fCurAngle += _fGeomSpinSpeed*fTimeElapsed;
        if( _fCurAngle > 2*PI )
            _fCurAngle -= 2*PI;
        if( fTimeToScan <= fTimeElapsed ) {
            // have to update
            SetTransform(_trans);
        }
    }

    return BaseLaser2DSensor::SimulationStep(fTimeElapsed);
}

SensorBase::SensorGeometry* BaseSpinningLaser2DSensor::GetSensorGeometry()
{
    SpinningLaserGeomData* pgeom = new SpinningLaserGeomData();
    *(LaserGeomData*)pgeom = _geom;
    pgeom->fSpinSpeed = _fGeomSpinSpeed;
    pgeom->vSpinAxis = _vGeomSpinAxis;
    pgeom->vSpinPos = _vGeomSpinPos;
    return pgeom;
}

bool BaseSpinningLaser2DSensor::SendCmd(std::istream& is, std::ostream& os)
{
    string cmd;
    stringstream::streampos pos = is.tellg();

    is >> cmd;

    if( !is )
        return false;

    if( stricmp(cmd.c_str(), "spin") == 0 ) {
        is >> _bSpinning;
        SetTransform(_trans);
    }
    else {
        is.seekg(pos);
        return BaseLaser2DSensor::SendCmd(is, os);
    }

    return !!is;
}

bool BaseSpinningLaser2DSensor::SupportsCmd(const char* pcmd)
{
    if( pcmd == NULL )
        return false;
    return stricmp(pcmd, "spin") == 0 || BaseLaser2DSensor::SupportsCmd(pcmd);
}
    
Transform BaseSpinningLaser2DSensor::GetLaserPlaneTransform()
{
    Transform trot;
    trot.rotfromaxisangle(_vGeomSpinAxis, _fCurAngle);
    trot.trans = trot.rotate(-_vGeomSpinPos) + _vGeomSpinPos;
    return GetTransform() * trot;
}
