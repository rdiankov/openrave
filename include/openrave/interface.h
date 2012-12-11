// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file   interface.h
    \brief  Base interface definition that all exported interfaces derive from.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_INTERFACE_BASE
#define OPENRAVE_INTERFACE_BASE

namespace OpenRAVE {

/// serialization options for interfaces
enum SerializationOptions
{
    SO_Kinematics = 0x01, ///< kinematics information
    SO_Dynamics = 0x02, ///< dynamics information
    SO_BodyState = 0x04, ///< state of the body
    SO_NamesAndFiles = 0x08, ///< resource files and names
    SO_RobotManipulators = 0x10, ///< serialize robot manipulators
    SO_RobotSensors = 0x20, ///< serialize robot sensors
    SO_Geometry = 0x40, ///< geometry information (for collision detection)
};

/** \brief <b>[interface]</b> Base class for all interfaces that OpenRAVE provides. See \ref interface_concepts.
    \ingroup interfaces
 */
class OPENRAVE_API InterfaceBase : public boost::enable_shared_from_this<InterfaceBase>
{
public:
    typedef std::map<std::string, XMLReadablePtr, CaseInsensitiveCompare> READERSMAP;

    InterfaceBase(InterfaceType type, EnvironmentBasePtr penv);
    virtual ~InterfaceBase();

    inline InterfaceType GetInterfaceType() const {
        return __type;
    }

    /// set internally by RaveDatabase
    /// \return the unique identifier that describes this class type, case is ignored
    /// should be the same id used to create the object
    inline const std::string& GetXMLId() const {
        return __strxmlid;
    }

    /// set internally by RaveDatabase
    /// \return the pluginname this interface was loaded from
    inline const std::string& GetPluginName() const {
        return __strpluginname;
    }

    /// \return the environment that this interface is attached to
    inline EnvironmentBasePtr GetEnv() const {
        return __penv;
    }

    inline const READERSMAP& GetReadableInterfaces() const {
        return __mapReadableInterfaces;
    }

    /// \brief returns the readable interface
    virtual XMLReadablePtr GetReadableInterface(const std::string& xmltag) const;

    /// \brief set a new readable interface and return the previously set interface if it exists
    virtual XMLReadablePtr SetReadableInterface(const std::string& xmltag, XMLReadablePtr readable);

    /// \brief Documentation of the interface in reStructuredText format. See \ref writing_plugins_doc.
    virtual const std::string& GetDescription() const {
        return __description;
    }

    virtual void SetDescription(const std::string& description) {
        __description = description;
    }

    /// \brief set user data for a specific key
    virtual void SetUserData(const std::string& key, UserDataPtr data) {
        __mapUserData[key] = data;
    }
    /// \brief return the user custom data
    virtual UserDataPtr GetUserData(const std::string& key=std::string()) const
    {
        std::map<std::string, UserDataPtr>::const_iterator it = __mapUserData.find(key);
        if( it == __mapUserData.end() ) {
            return UserDataPtr();
        }
        return it->second;
    }

    /// \brief removes a user data pointer. if user data pointer does not exist, then return 0, otherwise 1.
    virtual bool RemoveUserData(const std::string& key) {
        return __mapUserData.erase(key)>0;
    }

    /// \deprecated (12/12/11)
    virtual void SetUserData(UserDataPtr data) RAVE_DEPRECATED {
        __mapUserData[std::string()] = data;
    }

    /// \brief the URI used to load the interface (sometimes this is not possible if the definition lies inside an environment file).
    virtual const std::string& GetURI() const {
        return __struri;
    }
    virtual const std::string& GetXMLFilename() const {
        return __struri;
    }

    /// \brief Clone the contents of an interface to the current interface.
    ///
    /// \param preference the interface whose information to clone
    /// \param cloningoptions mask of CloningOptions
    /// \throw openrave_exception if command doesn't succeed
    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    /** \brief Used to send special commands to the interface and receive output.

        The command must be registered by \ref RegisterCommand. A special command '\b help' is
        always supported and provides a way for the user to query the current commands and the help
        string. The format of the returned help commands are in reStructuredText. The following commands are possible:
        - '\b help [command name]' - get the help string of just that command.
        - '\b help commands' - return the names of all the possible commands

        \param is the input stream containing the command
        \param os the output stream containing the output
        \exception openrave_exception Throw if the command is not supported.
        \return true if the command is successfully processed, otherwise false.
     */
    virtual bool SendCommand(std::ostream& os, std::istream& is);

    /** \brief serializes the interface

        The readable interfaces are also serialized within the tag, for example:

        \code{.xml}
        <sometag> <!-- root writer -->
          <interface> <!-- first child -->
            <readableinterface/> <!-- readable interface -->
          </interface>
        </sometag>
        \endcode

        Depending on the writer format, extra tags might be created.
     */
    virtual void Serialize(BaseXMLWriterPtr writer, int options=0) const;

protected:
    /// \brief The function to be executed for every command.
    ///
    /// \param sinput - input of the command
    /// \param sout - output of the command
    /// \return If false, there was an error with the command, true if successful
    typedef boost::function<bool (std::ostream&, std::istream&)> InterfaceCommandFn;
    class OPENRAVE_API InterfaceCommand
    {
public:
        InterfaceCommand() {
        }
        InterfaceCommand(InterfaceCommandFn newfn, const std::string& newhelp) : fn(newfn), help(newhelp) {
        }
        InterfaceCommandFn fn; ///< command function to run
        std::string help; ///< help string explaining command arguments
    };

    /// \brief Registers a command and its help string.
    ///
    /// \param cmdname - command name, converted to lower case
    /// \param fncmd function to execute for the command
    /// \param strhelp - help string in reStructuredText, see \ref writing_plugins_doc.
    /// \exception openrave_exception Throw if there exists a registered command already.
    virtual void RegisterCommand(const std::string& cmdname, InterfaceCommandFn fncmd, const std::string& strhelp);

    /// \brief Unregisters the command.
    virtual void UnregisterCommand(const std::string& cmdname);

    virtual const char* GetHash() const = 0;
    std::string __description;     /// \see GetDescription()

private:
    /// Write the help commands to an output stream
    virtual bool _GetCommandHelp(std::ostream& sout, std::istream& sinput) const;

    inline InterfaceBase& operator=(const InterfaceBase&r) {
        throw openrave_exception("InterfaceBase copying not allowed");
    }

    mutable boost::mutex _mutexInterface; ///< internal mutex for protecting data from methods that might be access from any thread (those methods should be commented).
    InterfaceType __type; ///< \see GetInterfaceType
    UserDataPtr __plugin; ///< handle to plugin that controls the executable code. As long as this plugin pointer is present, module will not be unloaded.
    std::string __struri; ///< \see GetURI
    std::string __strpluginname; ///< the name of the plugin, necessary?
    std::string __strxmlid; ///< \see GetXMLId
    EnvironmentBasePtr __penv; ///< \see GetEnv
    std::map<std::string, UserDataPtr> __mapUserData; ///< \see GetUserData

    READERSMAP __mapReadableInterfaces; ///< pointers to extra interfaces that are included with this object
    typedef std::map<std::string, boost::shared_ptr<InterfaceCommand>, CaseInsensitiveCompare> CMDMAP;
    CMDMAP __mapCommands; ///< all registered commands

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
    friend class OpenRAVEXMLParser::InterfaceXMLReader;
    friend class ColladaReader;
    friend class XFileReader;
#else
    friend class ::Environment;
    friend class ::OpenRAVEXMLParser::InterfaceXMLReader;
    friend class ::ColladaReader;
    friend class ::XFileReader;
#endif
#endif
    friend class RaveDatabase;
};

} // end namespace OpenRAVE

#endif
