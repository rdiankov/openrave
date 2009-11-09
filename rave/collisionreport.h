// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_COLLISIONREPORT_H
#define OPENRAVE_COLLISIONREPORT_H

namespace OpenRAVE {

/// OpenRAVE collision report
class COLLISIONREPORT
{
public:
    COLLISIONREPORT() { Reset(); }

    struct CONTACT
    {
        CONTACT() : depth(0) {}
        CONTACT(const Vector& p, const Vector& n, const dReal& d) : pos(p), norm(n) {depth = d;}

        Vector pos;             ///< where the contact occured
        Vector norm;    ///< the normals of the faces
        dReal depth;    ///< the penetration depth
    };

    int options;        ///< the options that the COLLISIONREPORT was called with

    /// Always filled
    KinBody::LinkConstPtr plink1, plink2; ///< colliding links
    int numCols;            ///< this is the number of objects that collide with the object of interest
    std::vector<KinBody::LinkConstPtr> vLinkColliding; ///< objects colliding with this object

    /// CO_Distance
    dReal minDistance;      ///< minimum distance from last query

    /// CO_UseTolerance
    int numWithinTol;       ///< number of objects within tolerance of this object

    /// CO_Contacts
    std::vector<CONTACT> contacts; ///< the convention is that the normal will be "out" of plink1's surface

    void Reset(int coloptions = 0) {
        options = coloptions;
        minDistance = 1e20f;
        numCols = 0;
        numWithinTol = 0;
        contacts.resize(0);
        vLinkColliding.resize(0);
    }
};

} // end namespace OpenRAVE

#endif
