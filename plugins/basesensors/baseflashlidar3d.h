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
#ifndef OPENRAVE_BASEFLASHLIDAR_H
#define OPENRAVE_BASEFLASHLIDAR_H

/// Flash LIDAR - sends laser points given a camera projection matrix
class BaseFlashLidar3DSensor : public SensorBase
{
protected:
    class BaseFlashLidar3DXMLReader : public BaseXMLReader
    {
    public:
        BaseFlashLidar3DXMLReader(BaseFlashLidar3DSensor* psensor) : _psensor(psensor) {}
        virtual ~BaseFlashLidar3DXMLReader() { delete _psensor; }

        virtual void* Release() { void* temp = _psensor; _psensor = NULL; return temp; }
        
        virtual void startElement(void *ctx, const char *name, const char **atts);
        virtual bool endElement(void *ctx, const char *name);
        virtual void characters(void *ctx, const char *ch, int len);

    protected:
        BaseFlashLidar3DSensor* _psensor;
        stringstream ss;
    };

    class BaseFlashLidar3DGeom : public LaserGeomData
    {
    public:
        float KK[4]; // intrinsic matrix expanding to [ KK[0] 0 KK[2]; 0 KK[1] KK[3] ]
        int width, height; // dimensions in number of lasers
    };

public:
    BaseFlashLidar3DSensor(EnvironmentBase* penv);
    ~BaseFlashLidar3DSensor();
    
    virtual bool Init(const char* args = NULL);
    virtual void Reset(int options);

    virtual bool SimulationStep(dReal fTimeElapsed);

    virtual SensorGeometry* GetSensorGeometry();

    virtual SensorData* CreateSensorData();
    virtual bool GetSensorData(SensorData* psensordata);

    virtual bool SendCmd(std::istream& is, std::ostream& os);
    virtual bool SupportsCmd(const char* pcmd);

    virtual void SetTransform(const Transform& trans);
    virtual Transform GetTransform() { return _trans; }

    virtual BaseXMLReader* CreateXMLReader() { return new BaseFlashLidar3DXMLReader(this); }
protected:

    BaseFlashLidar3DGeom _geom;
    LaserSensorData _data;
    vector<int> _databodyids; ///< if non 0, for each point in _data, specifies the body that was hit

    // more geom stuff
    dReal _fGeomMinRange;
    RaveVector<float> _vColor;
    float _iKK[4]; // inverse of KK

    Transform _trans;
    list<void*> _listGraphicsHandles;
    void* _pIconHandle;
    vector<RaveVector<float> > viconpoints;
    vector<int> viconindices;
    int _globid;
    dReal fTimeToScan, fScanTime;

    mutable pthread_mutex_t _mutexdata;

    bool _bRender;

    friend class BaseFlashLidar3DXMLReader;
};

#endif
