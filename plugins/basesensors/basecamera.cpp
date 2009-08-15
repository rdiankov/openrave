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
// along with this program.  If not, see <http://www.gnu.org/licenses/>.s/>.
#include "plugindefs.h"
#include "basecamera.h"

//////////////////////////////////
/// Regular Perspective Camera ///
//////////////////////////////////

void BaseCameraSensor::BaseCameraXMLReader::startElement(void *ctx, const char *name, const char **atts)
{
}

bool BaseCameraSensor::BaseCameraXMLReader::endElement(void *ctx, const char *name)
{    
    if( stricmp((const char*)name, "sensor") == 0 ) {
        return true;
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
    else if( stricmp((const char*)name, "framerate") == 0 ) {
        ss >> _psensor->framerate;
    }
    else if( stricmp((const char*)name, "power") == 0 ) {
        ss >> _psensor->_bPower;
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
        RAVEPRINT(L"BaseCameraSensor error parsing %s\n", name);

    return false;
}

void BaseCameraSensor::BaseCameraXMLReader::characters(void *ctx, const char *ch, int len)
{
    if( len > 0 ) {
        ss.clear();
        ss.str(string(ch, len));
    }
    else
        ss.str(""); // reset
}

BaseCameraSensor::BaseCameraSensor(EnvironmentBase* penv) : SensorBase(penv)
{
    _bShowCameraImage = false;
    _bPower = true;
    _pIconHandle = NULL;
    _vColor = RaveVector<float>(0.5f,0.5f,1,1);
    pthread_mutex_init(&_mutexdata, NULL);
    framerate = 5;
    _bUpdateCameraPlot = true;
}

BaseCameraSensor::~BaseCameraSensor()
{
    Reset(0);
    pthread_mutex_destroy(&_mutexdata);
}

bool BaseCameraSensor::Init(const char* args)
{
    stringstream ss(args != NULL ? args : "");

    Reset(0);

    return true;
}

void BaseCameraSensor::Reset(int options)
{
    if( _pIconHandle != NULL ) {
        GetEnv()->closegraph(_pIconHandle);
        _pIconHandle = NULL;
    }

    _data.vimagedata.resize(3*_geom.width*_geom.height);
    vimagedata.resize(3*_geom.width*_geom.height);
    fTimeToImage = 0;
    _globid = 0;
    _bUpdateCameraPlot = true;
}

bool BaseCameraSensor::SimulationStep(dReal fTimeElapsed)
{
    if( 0&&_bUpdateCameraPlot ) {
        // render a simple frustum outlining camera's dimension
        // the frustum is colored with vColor, the x and y axes are colored separetely
        Vector points[7];
        float ik0 = 1/_geom.KK[0], ik1 = 1/_geom.KK[1];
        points[0] = Vector(0,0,0);
        points[1] = Vector(((float)_geom.width-_geom.KK[2])*ik0, ((float)_geom.height-_geom.KK[3])*ik1, 1);
        points[2] = Vector(-_geom.KK[2]*ik0, ((float)_geom.height-_geom.KK[3])*ik1, 1);
        points[3] = Vector(-_geom.KK[2]*ik0, -_geom.KK[3]*ik1, 1);
        points[4] = Vector(((float)_geom.width-_geom.KK[2])*ik0, -_geom.KK[3]*ik1, 1);
        points[5] = Vector(0.5f,0,0);
        points[6] = Vector(0,0.5f,0);
    
        int inds[] = {0,1,2,3,4,1,4,0,2,3,0,0,5,0,0,6};

        viconpoints.resize(ARRAYSIZE(inds));
        vector<float> vcolors(ARRAYSIZE(inds)*3);

        for(unsigned int i = 0; i < ARRAYSIZE(inds); ++i) {
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
        
        void *newhandle = GetEnv()->drawlinestrip(viconpoints[0], viconpoints.size(), sizeof(viconpoints[0]), 1, &vcolors[0]);

        if( _pIconHandle != NULL )
            GetEnv()->closegraph(_pIconHandle);
        _pIconHandle = newhandle;
    }

    if( _geom.width > 0 && _geom.height > 0 && _bPower) {

        fTimeToImage -= fTimeToImage;
        if( fTimeToImage <= 0 ) {
            fTimeToImage = 1 / (float)framerate;

            // releasing the physics lock is a big problem because the environment or other functions can get control and destroy this sensor!!
            GetEnv()->LockPhysics(false);
            if( !GetEnv()->GetCameraImage(&vimagedata[0], _geom.width, _geom.height, _trans, _geom.KK) )
                RAVELOG(L"camera failed to get image\n");
            //GetEnv()->WriteCameraImage(_geom.width, _geom.height, _trans, _geom.KK,"cam.jpg","jpg");
            GetEnv()->LockPhysics(true);

            // copy the data
            MutexLock m(&_mutexdata); // can crash here if sensor was destroyed
            _data.t = _trans;
            _data.vimagedata = vimagedata;
            _data.id = ++_globid;
        }
    }
        
    return true;
}

SensorBase::SensorGeometry* BaseCameraSensor::GetSensorGeometry()
{
    CameraGeomData* pgeom = new CameraGeomData();
    *pgeom = _geom;
    return pgeom;
}

SensorBase::SensorData* BaseCameraSensor::CreateSensorData()
{
    return new CameraSensorData();
}

bool BaseCameraSensor::GetSensorData(SensorData* psensordata)
{
    if( psensordata == NULL || psensordata->GetType() != ST_Camera )
        return false;

    MutexLock m(&_mutexdata);
    *(CameraSensorData*)psensordata = _data;
    return true;
}

bool BaseCameraSensor::SendCmd(std::istream& is, std::ostream& os)
{
    string cmd;
    is >> cmd;

    if( !is )
        return false;

    if( stricmp(cmd.c_str(), "show") == 0 )
        is >> _bShowCameraImage;
    else if( stricmp(cmd.c_str(),"power") == 0 )
        is >> _bPower;
    else if( stricmp(cmd.c_str(), "setintrinsic") == 0 )
        is >> _geom.KK[0] >> _geom.KK[1] >> _geom.KK[2] >> _geom.KK[3];
    else if( stricmp(cmd.c_str(), "setdims") == 0 ) {
        is >> _geom.width >> _geom.height;
        Reset(0);
    }

    return !!is;
}

bool BaseCameraSensor::SupportsCmd(const char* pcmd)
{
    if( pcmd == NULL )
        return false;
    return stricmp(pcmd, "show") == 0 || stricmp(pcmd, "setintrinsic") == 0 ||
        stricmp(pcmd, "setdims") == 0 || stricmp(pcmd, "power") == 0;
}

void BaseCameraSensor::SetTransform(const Transform& trans)
{
    _trans = trans;
    _bUpdateCameraPlot = true;
}

bool BaseCameraSensor::Clone(const InterfaceBase* preference, int cloningoptions)
{
    if( preference == NULL || strcmp(GetXMLId(),preference->GetXMLId()) )
        return false;
    const BaseCameraSensor& r = *(const BaseCameraSensor*)preference;
    _geom = r._geom;
    _vColor = r._vColor;
    _trans = r._trans;
    _globid = r._globid;
    fTimeToImage = r.fTimeToImage;
    framerate = r.framerate;
    _bShowCameraImage = r._bShowCameraImage;
    _bUpdateCameraPlot = r._bUpdateCameraPlot;
    _bPower = r._bUpdateCameraPlot;
    Reset(0);
    return true;
}
