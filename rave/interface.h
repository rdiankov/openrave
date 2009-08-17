// Copyright (C) 2006-2009 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_INTERFACE_BASE
#define OPENRAVE_INTERFACE_BASE

namespace OpenRAVE {

/// base class for all interfaces that OpenRAVE provides
class InterfaceBase
{
public:
    InterfaceBase(PluginType type, EnvironmentBase* penv) : __type(type), __penv(penv), __pUserData(NULL) {}
	virtual ~InterfaceBase() {
        for(std::map<std::string, XMLReadable* >::iterator it = __mapReadableInterfaces.begin(); it != __mapReadableInterfaces.end(); ++it) {
            delete it->second;
        }
        __mapReadableInterfaces.clear();
    }

    inline PluginType GetInterfaceType() const { return __type; }

    /// set internally by RaveDatabase
	/// \return the unique identifier that describes this class type, case is ignored
    /// should be the same id used to create the object
    inline const char* GetXMLId() const { return __strxmlid.c_str(); }

    /// set internally by RaveDatabase
    /// \return the pluginname this interface was loaded from
    inline const char* GetPluginName() const { return __strpluginname.c_str(); }

    /// \return the environment that this interface is attached to
    inline EnvironmentBase* GetEnv() const { return __penv; }

    inline const std::map<std::string, XMLReadable* >& GetReadableInterfaces() const { return __mapReadableInterfaces; }
    inline XMLReadable* GetReadableInterface(const std::string& xmltag) const
    {
        std::map<std::string, XMLReadable* >::const_iterator it = __mapReadableInterfaces.find(xmltag);
        return it != __mapReadableInterfaces.end() ? it->second : NULL;
    }

    virtual void SetUserData(void* pdata) { __pUserData = pdata; }
    virtual void* GetUserData() const { return __pUserData; }
    
    /// clone the contents of an interface to the current interface
    /// \param preference the interface whose information to clone
    /// \param cloningoptions mask of CloningOptions
    virtual bool Clone(const InterfaceBase* preference, int cloningoptions) { return true; }

protected:
    virtual const char* GetHash() const = 0;

private:
    std::string __strpluginname, __strxmlid;
    PluginType __type;
    EnvironmentBase* __penv;
    void* __pUserData;                       ///< data set by the user
    std::map<std::string, XMLReadable* > __mapReadableInterfaces; ///< pointers to extra interfaces that are included with this object

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class RaveDatabase;
    friend class InterfaceXMLReader;
#else
    friend class ::RaveDatabase;
    friend class ::InterfaceXMLReader;
#endif
#endif
};

} // end namespace OpenRAVE

#endif
