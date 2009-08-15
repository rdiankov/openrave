// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_SENSORSYSTEM_H
#define OPENRAVE_SENSORSYSTEM_H

namespace OpenRAVE {

/// used to manage objects through a sensor system.
///
/// New objects can be created, existing objects can be updated.
class SensorSystemBase : public InterfaceBase
{
public:
    class BODYBASE
    {
    public:
        virtual ~BODYBASE() {}

        /// returns a pointer to the data used to initialize the BODY with AddKinBody.
        /// if psize is not NULL, will be filled with the size of the data in bytes
        /// This function will be used to restore bodies that were removed
        virtual void* GetInitData(int* psize) const = 0;

        /// particular link that sensor system is tracking.
        /// All transformations describe this link.
        virtual KinBody::Link* GetOffsetLink() const = 0;

        /// true if the object is being updated by the system due to its presence in the real environment
        virtual bool IsPresent() const = 0;

        /// true if should update openrave body
        virtual bool IsEnabled() const = 0;

        /// if true, the vision system should not destroy this object once it stops being present
        virtual bool IsLocked() const = 0;

        /// set a lock on a particular body
        virtual bool Lock(bool bDoLock) = 0;
    };

    SensorSystemBase(EnvironmentBase* penv) : InterfaceBase(penv) {}
    virtual ~SensorSystemBase() {}

    /// initializes the sensor system
    /// \param args string of arguments
    virtual bool Init(std::istream& sinput) = 0;
    virtual void Destroy() = 0;

    /// automatically register bodies that have some type of SensorSystem data (usually done through xml)
    virtual void AddRegisteredBodies(const std::vector<KinBody*>& vbodies) = 0;

    /// add body for registering with sensor system
    /// pdata is a pointer to a data structor holding tracking/registration information for the system
    virtual BODYBASE*  AddKinBody(KinBody* pbody, const void* pdata) = 0;
    /// remove body from sensory system. If bDestroy is true, will also deallocate the memory
    virtual bool RemoveKinBody(KinBody* pbody) = 0;
    /// returns true if body is present
    virtual bool IsBodyPresent(KinBody* pbody) = 0;
    /// enable/disable a body from being updated by the sensor system
    virtual bool EnableBody(KinBody* pbody, bool bEnable) = 0;
    /// get a pointer to the BODYBASE struct (can be also used for checking inclusion)
    virtual BODYBASE* GetBody(KinBody* pbody) = 0;

    /// switches the registrations of two bodies. Can be used to quickly change the models of the current bodies
    /// \param pbody1 First body to switch
    /// \param pbody2 Second body to switch
    virtual bool SwitchBody(KinBody* pbody1, KinBody* pbody2) = 0;

private:
    virtual const char* GetHash() const { return OPENRAVE_SENSORSYSTEM_HASH; }
};

} // end namespace OpenRAVE

#endif
