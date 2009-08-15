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

/// Laser rotates around the zaxis and it's 0 angle is pointed toward the xaxis.
class BaseCameraSensor : public SensorBase
{
protected:
    class BaseCameraXMLReader : public BaseXMLReader
    {
    public:
        BaseCameraXMLReader(BaseCameraSensor* psensor) : _psensor(psensor) {}
        virtual ~BaseCameraXMLReader() { delete _psensor; }

        virtual void* Release() { void* temp = _psensor; _psensor = NULL; return temp; }
        
        virtual void startElement(void *ctx, const char *name, const char **atts);
        virtual bool endElement(void *ctx, const char *name);
        virtual void characters(void *ctx, const char *ch, int len);

    protected:
        BaseCameraSensor* _psensor;
        stringstream ss;
    };

public:
    BaseCameraSensor(EnvironmentBase* penv);
    ~BaseCameraSensor();
    
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

    virtual BaseXMLReader* CreateXMLReader() { return new BaseCameraXMLReader(this); }

    virtual bool Clone(const InterfaceBase* preference, int cloningoptions);

protected:

    CameraGeomData _geom;
    CameraSensorData _data;

    // more geom stuff
    vector<char> vimagedata;
    RaveVector<float> _vColor;

    Transform _trans;
    dReal fTimeToImage;
    int framerate;
    void* _pIconHandle;
    vector<RaveVector<float> > viconpoints;
    int _globid;

    mutable pthread_mutex_t _mutexdata;

    bool _bShowCameraImage; ///< if true, will show the camera image
    bool _bUpdateCameraPlot;
    bool _bPower; ///< if true, gather data, otherwise don't

    friend class BaseCameraXMLReader;
};

#endif
