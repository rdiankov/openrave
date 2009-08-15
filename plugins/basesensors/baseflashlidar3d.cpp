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
#include "baseflashlidar3d.h"

////////////////////////
/// Regular 3D Laser ///
////////////////////////

void BaseFlashLidar3DSensor::BaseFlashLidar3DXMLReader::startElement(void *ctx, const char *name, const char **atts)
{
}

bool BaseFlashLidar3DSensor::BaseFlashLidar3DXMLReader::endElement(void *ctx, const char *name)
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
    else if( stricmp((const char*)name, "KK") == 0 ) {
        for(int i = 0; i < 4; ++i)
            ss >> _psensor->_geom.KK[i];
    }
    else if( stricmp((const char*)name, "width") == 0 ) {
        ss >> _psensor->_geom.width;
    }
    else if( stricmp((const char*)name, "height") == 0 ) {
        ss >> _psensor->_geom.height;
    }
    else
        RAVEPRINT(L"unknown field %s\n", name);

    if( !ss )
        RAVEPRINT(L"BaseFlashLidar3DSensor error parsing %s\n", name);

    return false;
}

void BaseFlashLidar3DSensor::BaseFlashLidar3DXMLReader::characters(void *ctx, const char *ch, int len)
{
    if( len > 0 ) {
        ss.clear();
        ss.str(string(ch, len));
    }
    else
        ss.str(""); // reset
}

BaseFlashLidar3DSensor::BaseFlashLidar3DSensor(EnvironmentBase* penv) : SensorBase(penv)
{
    _bRender = false;
    _geom.min_angle[0] = -PI/2; _geom.min_angle[1] = 0;
    _geom.max_angle[0] = PI/2; _geom.max_angle[1] = 0;
    _geom.max_range = 100;
    _geom.KK[0] = 500; _geom.KK[1] = 500; _geom.KK[2] = 250; _geom.KK[3] = 250;
    _geom.width = 64; _geom.height = 64;
    fTimeToScan = 0;
    _pIconHandle = NULL;
    _vColor = RaveVector<float>(0.5f,0.5f,1,1);
    pthread_mutex_init(&_mutexdata, NULL);
}

BaseFlashLidar3DSensor::~BaseFlashLidar3DSensor()
{
    Reset(0);
    pthread_mutex_destroy(&_mutexdata);
}

bool BaseFlashLidar3DSensor::Init(const char* args)
{
    stringstream ss(args != NULL ? args : "");

    _iKK[0] = 1.0f / _geom.KK[0];
    _iKK[1] = 1.0f / _geom.KK[1];
    _iKK[2] = -_geom.KK[2] / _geom.KK[0];
    _iKK[3] = -_geom.KK[3] / _geom.KK[1];
    
    Reset(0);

    return true;
}

void BaseFlashLidar3DSensor::Reset(int options)
{
    FOREACH(it, _listGraphicsHandles)
        GetEnv()->closegraph(*it);
    _listGraphicsHandles.clear();

    if( _pIconHandle != NULL ) {
        GetEnv()->closegraph(_pIconHandle);
        _pIconHandle = NULL;
    }

    _data.positions.clear();
    _data.ranges.resize(_geom.width*_geom.height);
    _data.intensity.resize(_geom.width*_geom.height);
    _databodyids.resize(_geom.width*_geom.height);

    FOREACH(it, _data.ranges)
        *it = Vector(0,0,0);
    FOREACH(it, _data.intensity)
        *it = 0;

    _globid = 0;
}

bool BaseFlashLidar3DSensor::SimulationStep(dReal fTimeElapsed)
{
    fTimeToScan -= fTimeElapsed;
    if( fTimeToScan <= 0 ) {
        fTimeToScan = fScanTime;

        RAY r;
    
        GetEnv()->SetCollisionOptions(CO_Distance);
        COLLISIONREPORT report;    
        Transform t;

        {
            // Lock the data mutex and fill with the range data (get all in one timestep)
            MutexLock m(&_mutexdata);
            _data.t = GetTransform();
            _data.id = ++_globid;
        
            t = GetTransform();
            r.pos = t.trans;

            for(int w = 0; w < _geom.width; ++w) {
                for(int h = 0; h < _geom.height; ++h) {
                    r.dir.x = (float)w*_iKK[0] + _iKK[2];
                    r.dir.y = (float)h*_iKK[1] + _iKK[3];
                    r.dir.z = 1.0f;
                    r.dir = _geom.max_range*_data.t.rotate(r.dir.normalize3());

                    int index = w*_geom.height+h;

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
            
            // render the transparent fan for every column
            vindices.resize(3*(_geom.height-1)*_geom.width);
            int index = 0;
            for(int w = 0; w < _geom.width; ++w) {
                for(int i = 0; i < _geom.height-1; ++i) {
                    vindices[index++] = w*_geom.height+i;
                    vindices[index++] = w*_geom.height+i+1;
                    vindices[index++] = N;
                }
            }

            _vColor.w = 1;
            // Render points at each measurement, and a triangle fan for the entire free surface of the laser
            listhandles.push_back(GetEnv()->plot3(&vpoints[0].x, N, sizeof(vpoints[0]), 5.0f, _vColor));
            
            _vColor.w = 0.01f;
            listhandles.push_back(GetEnv()->drawtrimesh(vpoints[0], sizeof(vpoints[0]), &vindices[0], vindices.size()/3, _vColor));
            
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

SensorBase::SensorGeometry* BaseFlashLidar3DSensor::GetSensorGeometry()
{
    BaseFlashLidar3DGeom* pgeom = new BaseFlashLidar3DGeom();
    *pgeom = _geom;
    return pgeom;
}

SensorBase::SensorData* BaseFlashLidar3DSensor::CreateSensorData()
{
    return new LaserSensorData();
}

bool BaseFlashLidar3DSensor::GetSensorData(SensorData* psensordata)
{
    if( psensordata == NULL || psensordata->GetType() != ST_Laser )
        return false;

    MutexLock m(&_mutexdata);
    *(LaserSensorData*)psensordata = _data;
    return true;
}

bool BaseFlashLidar3DSensor::SendCmd(std::istream& is, std::ostream& os)
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

bool BaseFlashLidar3DSensor::SupportsCmd(const char* pcmd)
{
    if( pcmd == NULL )
        return false;
    return stricmp(pcmd, "show") == 0 || stricmp(pcmd, "render") == 0 || stricmp(pcmd, "collidingbodies") == 0;
}

void BaseFlashLidar3DSensor::SetTransform(const Transform& trans)
{
    _trans = trans;

    // draw new graph
    if( _pIconHandle != NULL ) {
        GetEnv()->closegraph(_pIconHandle);
        _pIconHandle = NULL;
    }

    Transform t = GetTransform();
    
    viconpoints.resize(5);
    viconindices.resize(6*3);
    viconpoints[0] = t.trans;
    viconpoints[1] = t.trans+0.1f*t.rotate(Vector(_iKK[2], _iKK[3],1));
    viconpoints[2] = t.trans+0.1f*t.rotate(Vector(_iKK[2], (float)_geom.height*_iKK[1] + _iKK[3],1));
    viconpoints[3] = t.trans+0.1f*t.rotate(Vector((float)_geom.width*_iKK[0] + _iKK[2], _iKK[3],1));
    viconpoints[4] = t.trans+0.1f*t.rotate(Vector((float)_geom.width*_iKK[0] + _iKK[2], (float)_geom.height*_iKK[1] + _iKK[3],1));

    viconindices[0] = 0; viconindices[1] = 1; viconindices[2] = 2;
    viconindices[3] = 0; viconindices[4] = 4; viconindices[5] = 2;
    viconindices[6] = 0; viconindices[7] = 3; viconindices[8] = 4;
    viconindices[9] = 0; viconindices[10] = 1; viconindices[11] = 3;
    viconindices[12] = 1; viconindices[13] = 3; viconindices[14] = 2;
    viconindices[15] = 3; viconindices[16] = 4; viconindices[17] = 2;

    RaveVector<float> vcolor = _vColor*0.5f;
    vcolor.w = 0.7f;
    _pIconHandle = GetEnv()->drawtrimesh(viconpoints[0], sizeof(viconpoints[0]), &viconindices[0], 6, vcolor);
}
