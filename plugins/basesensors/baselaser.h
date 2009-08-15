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
#ifndef OPENRAVE_BASELASER_H
#define OPENRAVE_BASELASER_H

/// Laser rotates around the zaxis and it's 0 angle is pointed toward the xaxis.
class BaseLaser2DSensor : public SensorBase
{
protected:
    class BaseLaser2DXMLReader : public BaseXMLReader
    {
    public:
        BaseLaser2DXMLReader(BaseLaser2DSensor* psensor) : _psensor(psensor) {}
        virtual ~BaseLaser2DXMLReader() { delete _psensor; }

        virtual void* Release() { void* temp = _psensor; _psensor = NULL; return temp; }
        
        virtual void startElement(void *ctx, const char *name, const char **atts);
        virtual bool endElement(void *ctx, const char *name);
        virtual void characters(void *ctx, const char *ch, int len);

    protected:
        BaseLaser2DSensor* _psensor;
        stringstream ss;
    };

public:
    BaseLaser2DSensor(EnvironmentBase* penv);
    ~BaseLaser2DSensor();
    
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

    virtual BaseXMLReader* CreateXMLReader() { return new BaseLaser2DXMLReader(this); }
protected:

    virtual Transform GetLaserPlaneTransform() { return _trans; }

    LaserGeomData _geom;
    LaserSensorData _data;
    vector<int> _databodyids; ///< if non 0, for each point in _data, specifies the body that was hit

    // more geom stuff
    dReal _fGeomMinRange;
    RaveVector<float> _vColor;

    Transform _trans;
    list<void*> _listGraphicsHandles;
    void* _pIconHandle;
    vector<RaveVector<float> > viconpoints;
    vector<int> viconindices;
    int _globid;
    dReal fTimeToScan, fScanTime;

    mutable pthread_mutex_t _mutexdata;

    bool _bRender;

    friend class BaseLaser2DXMLReader;
};

class BaseSpinningLaser2DSensor : public BaseLaser2DSensor
{
protected:
    class BaseSpinningLaser2DXMLReader : public BaseLaser2DXMLReader
    {
    public:
        BaseSpinningLaser2DXMLReader(BaseSpinningLaser2DSensor* psensor) : BaseLaser2DXMLReader(psensor) {}
        virtual bool endElement(void *ctx, const char *name);
    };

    class SpinningLaserGeomData : public LaserGeomData
    {
    public:
        dReal fSpinSpeed;
        Vector vSpinAxis, vSpinPos;
    };
    
public:
    BaseSpinningLaser2DSensor(EnvironmentBase* penv);
    
    virtual bool Init(const char* args = NULL);
    virtual void Reset(int options);

    virtual bool SimulationStep(dReal fTimeElapsed);

    virtual SensorGeometry* GetSensorGeometry();

    virtual bool SendCmd(std::istream& is, std::ostream& os);
    virtual bool SupportsCmd(const char* pcmd);

    virtual BaseXMLReader* CreateXMLReader() { return new BaseSpinningLaser2DXMLReader(this); }

protected:

    virtual Transform GetLaserPlaneTransform();

    dReal _fGeomSpinSpeed;
    Vector _vGeomSpinAxis;
    Vector _vGeomSpinPos;

    dReal _fCurAngle;
    bool _bSpinning;
};

#endif
