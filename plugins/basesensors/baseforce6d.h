// -*- coding: utf-8 -*-
// Copyright (C) 2020 Eisoku Kuroiwa <eisoku.kuroiwa@mujin.co.jp>
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
#ifndef OPENRAVE_BASEFORCE6D_H
#define OPENRAVE_BASEFORCE6D_H

#include <boost/lexical_cast.hpp>

class BaseForce6DSensor : public SensorBase
{
protected:
    class BaseForce6DXMLReader : public BaseXMLReader
    {
    public:
        BaseForce6DXMLReader(boost::shared_ptr<BaseForce6DSensor> psensor) : _psensor(psensor) {
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support ) {
                    return PE_Support;
                }
                return PE_Ignore;
            }
            static boost::array<string, 4> tags = { { "sensor", "polarity", "correction_matrix", "hardware_id"}};
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
            else if( name == "polarity" ) {
                ss >> _psensor->_pgeom->polarity;
            }
            else if( name == "correction_matrix" ) {
                for (size_t i = 0; i < _psensor->_pgeom->correction_matrix.size(); ++i) {
                    ss >> _psensor->_pgeom->correction_matrix[i];
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
        boost::shared_ptr<BaseForce6DSensor> _psensor;
        stringstream ss;
    };


    class BaseForce6DJSONReader : public BaseJSONReader
    {
    public:
        BaseForce6DJSONReader(ReadablePtr pReadable) {
            if (!!pReadable) {
                _pgeom = boost::dynamic_pointer_cast<Force6DGeomData>(pReadable);
            }
            else {
                _pgeom.reset(new Force6DGeomData());
            }
        }
        virtual ~BaseForce6DJSONReader() {}
        ReadablePtr GetReadable() override {
            return _pgeom;
        }
    protected:
        Force6DGeomDataPtr _pgeom;
    };

public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        return BaseXMLReaderPtr(new BaseForce6DXMLReader(boost::dynamic_pointer_cast<BaseForce6DSensor>(ptr)));
    }

    static BaseJSONReaderPtr CreateJSONReader(ReadablePtr pReadable, const AttributesList& atts)
    {
        return BaseJSONReaderPtr(new BaseForce6DJSONReader(pReadable));
    }

    BaseForce6DSensor(EnvironmentBasePtr penv) : SensorBase(penv) {
        __description = ":Interface Author: Eisoku Kuroiwa\n\nProvides a simulated force6D sensor.";
        _pgeom.reset(new Force6DGeomData());
        _pdata.reset(new Force6DSensorData());
    }

    virtual int Configure(ConfigureCommand command, bool blocking) override
    {
        return 1;
    }

    virtual void SetSensorGeometry(SensorGeometryConstPtr pgeometry) override
    {
        OPENRAVE_ASSERT_OP(pgeometry->GetType(), ==, ST_Force6D );
        *_pgeom = *boost::static_pointer_cast<Force6DGeomData const>(pgeometry);
    }

    virtual bool SimulationStep(dReal fTimeElapsed) override
    {
        return true;
    }

    virtual SensorGeometryConstPtr GetSensorGeometry(SensorType type) override
    {
        if(( type == ST_Invalid) ||( type == ST_Force6D) ) {
            Force6DGeomData* pgeom = new Force6DGeomData();
            *pgeom = *_pgeom;
            return SensorGeometryConstPtr(boost::shared_ptr<Force6DGeomData>(pgeom));
        }
        return SensorGeometryConstPtr();
    }

    virtual SensorDataPtr CreateSensorData(SensorType type) override
    {
        if(( type == ST_Invalid) ||( type == ST_Force6D) ) {
            return SensorDataPtr(boost::shared_ptr<Force6DSensorData>(new Force6DSensorData()));
        }
        return SensorDataPtr();
    }

    virtual bool GetSensorData(SensorDataPtr psensordata) override
    {
        if( psensordata->GetType() == ST_Force6D ) {
            std::lock_guard<std::mutex> lock(_mutexdata);
            *boost::dynamic_pointer_cast<Force6DSensorData>(psensordata) = *_pdata;
            return true;
        }
        return false;
    }

    virtual bool Supports(SensorType type) override {
        return type == ST_Force6D;
    }

    virtual void SetTransform(const Transform& trans) override
    {
        _trans = trans;
    }

    virtual const Transform& GetTransform() override {
        return _trans;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions) override
    {
        SensorBase::Clone(preference,cloningoptions);
        boost::shared_ptr<BaseForce6DSensor const> r = boost::dynamic_pointer_cast<BaseForce6DSensor const>(preference);
        *_pgeom = *r->_pgeom;
        _trans = r->_trans;
    }

    void Serialize(BaseXMLWriterPtr writer, int options=0) const override
    {
        _pgeom->SerializeXML(writer, options);
    }

protected:
    boost::shared_ptr<Force6DGeomData> _pgeom;
    boost::shared_ptr<Force6DSensorData> _pdata;

    Transform _trans;

    mutable std::mutex _mutexdata;

    friend class BaseForce6DXMLReader;
};

#endif
