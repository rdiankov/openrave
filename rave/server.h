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
#ifndef OPENRAVE_SERVER_H
#define OPENRAVE_SERVER_H

namespace OpenRAVE {

/// Base class for server engine
class RaveServerBase : public InterfaceBase
{
public:
    RaveServerBase(EnvironmentBase* penv) : InterfaceBase(PT_Server, penv) {}
    virtual ~RaveServerBase() {}

    virtual void Destroy() = 0;
    virtual void Reset() = 0;

    virtual bool Init(int port) = 0;

    /// worker thread called from the main environment thread
    virtual void Worker() = 0;

    virtual bool IsInit() = 0;
    virtual bool IsClosing() = 0;

private:
    virtual const char* GetHash() const { return OPENRAVE_SERVER_HASH; }
};

} // end namespace OpenRAVE

#endif
