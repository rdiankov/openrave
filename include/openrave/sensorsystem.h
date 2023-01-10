// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
/** \file sensorsystem.h
    \brief Sensor systems used to define how environment information is organized and how bodies are managed.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_SENSORSYSTEM_H
#define OPENRAVE_SENSORSYSTEM_H

#include <thread>

namespace OpenRAVE {

/** \brief <b>[interface]</b> Used to manage the creation and destruction of bodies. See \ref arch_sensorsystem.
    \ingroup interfaces
 */
class OPENRAVE_API SensorSystemBase : public InterfaceBase
{
public:
    SensorSystemBase(EnvironmentBasePtr penv) : InterfaceBase(PT_SensorSystem, penv) {
    }
    virtual ~SensorSystemBase() {
    }

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_SensorSystem;
    }

    /// resets the system and stops managing all objects. Any objects that are not locked, are deleted
    virtual void Reset() = 0;

    /// automatically register bodies that have some type of SensorSystem data (usually done through xml)
    virtual void AddRegisteredBodies(const std::vector<KinBodyPtr>& vbodies) = 0;

    /// add body for registering with sensor system
    /// pdata is a pointer to a data structor holding tracking/registration information for the system
    virtual KinBody::ManageDataPtr AddKinBody(KinBodyPtr pbody, ReadableConstPtr pdata) = 0;
    /// remove body from sensory system. If bDestroy is true, will also deallocate the memory
    virtual bool RemoveKinBody(KinBodyPtr pbody) = 0;
    /// returns true if body is present
    virtual bool IsBodyPresent(KinBodyPtr pbody) = 0;
    /// enable/disable a body from being updated by the sensor system
    virtual bool EnableBody(KinBodyPtr pbody, bool bEnable) = 0;

    /// switches the registrations of two bodies. Can be used to quickly change the models of the current bodies
    /// \param pbody1 First body to switch
    /// \param pbody2 Second body to switch
    virtual bool SwitchBody(KinBodyPtr pbody1, KinBodyPtr pbody2) = 0;

protected:
    virtual void SetManageData(KinBodyPtr pbody, KinBody::ManageDataPtr data) {
        pbody->SetManageData(data);
    }
};

/// A very simple sensor system example that manages raw detection data
class OPENRAVE_API SimpleSensorSystem : public SensorSystemBase
{
public:
    class OPENRAVE_API XMLData : public Readable {
public:
        XMLData(const std::string& xmlid) : Readable(xmlid), id(0) {
        }

        bool SerializeXML(BaseXMLWriterPtr writer, int options=0) const override {
            return false;
        }

        virtual void copy(boost::shared_ptr<XMLData const> pdata) {
            *this = *pdata;
        }

        /// \return true if this readable is equivalent to other readable
        virtual bool operator==(const Readable& other) const override {
            if (GetXMLId() != other.GetXMLId()) {
                return false;
            }
            const XMLData* pOther = dynamic_cast<const XMLData*>(&other);
            if (!pOther) {
                return false;
            }
            return sid == pOther->sid &&
                id == pOther->id &&
                strOffsetLink == pOther->strOffsetLink &&
                transOffset == pOther->transOffset &&
                transPreOffset == pOther->transPreOffset;
        }

        /// \return return a cloned copy of this readable
        virtual ReadablePtr CloneSelf() const override {
            boost::shared_ptr<XMLData> pNew(new XMLData(GetXMLId()));
            *pNew = *this;
            return pNew;
        }


        std::string sid;         ///< global id for the system id
        int id;
        std::string strOffsetLink;         ///< the link where the markers are attached (if any)
        Transform transOffset,transPreOffset;         // final offset = transOffset * transReturnedFromVision * transPreOffset

        friend class SimpleSensorSystem;
    };

    class OPENRAVE_API BodyData : public KinBody::ManageData {
public:
        BodyData(SensorSystemBasePtr psensorsystem, KinBodyPtr pbody, boost::shared_ptr<XMLData> initdata) : KinBody::ManageData(psensorsystem), _initdata(initdata), bPresent(false), bEnabled(true), bLock(false)
        {
            SetBody(pbody);
        }

        virtual ReadableConstPtr GetData() const {
            return _initdata;
        }
        virtual KinBody::LinkPtr GetOffsetLink() const {
            return KinBody::LinkPtr(_plink);
        }

        virtual bool IsPresent() const {
            return bPresent;
        }
        virtual bool IsEnabled() const {
            return bEnabled;
        }
        virtual bool IsLocked() const {
            return bLock;
        }
        virtual bool Lock(bool bDoLock) {
            bLock = bDoLock; return true;
        }

        virtual int GetId() {
            return _initdata->id;
        }
        virtual const std::string& GetSid() {
            return _initdata->sid;
        }
        virtual const Transform& GetRecentTransform() {
            return tnew;
        }

protected:
        virtual void SetBody(KinBodyPtr pbody)
        {
            KinBody::LinkPtr plink;
            if( _initdata->strOffsetLink.size() > 0 )
                plink = pbody->GetLink(_initdata->strOffsetLink);
            if( !plink )
                plink = pbody->GetLinks().front();
            _plink = plink;
        }

        boost::shared_ptr<XMLData> _initdata;
        uint64_t lastupdated;
        Transform tnew;         ///< most recent transform that is was set

        bool bPresent;
        bool bEnabled;
        bool bLock;

        KinBody::LinkWeakPtr _plink;
        friend class SimpleSensorSystem;
    };

    class OPENRAVE_API SimpleXMLReader : public BaseXMLReader
    {
public:
        SimpleXMLReader(boost::shared_ptr<XMLData>);
        virtual ReadablePtr GetReadable() {
            return _pdata;
        }
        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
        virtual bool endElement(const std::string& name);
        virtual void characters(const std::string& ch);

protected:
        boost::shared_ptr<XMLData> _pdata;
        std::stringstream ss;
    };

    /// registers the XML reader, do not call in the constructor of this class!
    static UserDataPtr RegisterXMLReaderId(EnvironmentBasePtr penv, const std::string& xmlid);

    SimpleSensorSystem(const std::string& xmlid, EnvironmentBasePtr penv);
    virtual ~SimpleSensorSystem();

    virtual void Reset();

    virtual void AddRegisteredBodies(const std::vector<KinBodyPtr>& vbodies);
    virtual KinBody::ManageDataPtr AddKinBody(KinBodyPtr pbody, ReadableConstPtr pdata);

    virtual bool RemoveKinBody(KinBodyPtr pbody);
    virtual bool IsBodyPresent(KinBodyPtr pbody);
    virtual bool EnableBody(KinBodyPtr pbody, bool bEnable);
    virtual bool SwitchBody(KinBodyPtr pbody1, KinBodyPtr pbody2);

protected:
    typedef std::pair<boost::shared_ptr<BodyData>, Transform > SNAPSHOT;
    typedef std::map<int,boost::shared_ptr<BodyData> > BODIES;
    virtual boost::shared_ptr<BodyData> CreateBodyData(KinBodyPtr pbody, boost::shared_ptr<XMLData const> pdata);
    virtual void _UpdateBodies(std::list<SNAPSHOT>& listbodies);
    virtual void _UpdateBodiesThread();

    virtual void SetRecentTransform(boost::shared_ptr<BodyData> pdata, const Transform& t) {
        pdata->tnew = t;
    }

    /// creates a reader to parse the data
    static BaseXMLReaderPtr CreateXMLReaderId(const std::string& xmlid, InterfaceBasePtr ptr, const AttributesList& atts);

    std::string _xmlid;
    BODIES _mapbodies;
    std::mutex _mutex;
    uint64_t _expirationtime;     ///< expiration time in us
    bool _bShutdown;
    std::thread _threadUpdate;
};

} // end namespace OpenRAVE

#endif
