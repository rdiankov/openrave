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
class RAVE_API CollisionReport
{
public:
    CollisionReport() { Reset(); }

    struct RAVE_API CONTACT
    {
        CONTACT() : depth(0) {}
        CONTACT(const Vector& p, const Vector& n, dReal d) : pos(p), norm(n) {depth = d;}

        Vector pos;             ///< where the contact occured
        Vector norm;    ///< the normals of the faces
        dReal depth;    ///< the penetration depth, positive means the surfaces are penetrating, negative means the surfaces are not colliding (used for distance queries)
    };

    int options;        ///< the options that the CollisionReport was called with

    KinBody::LinkConstPtr plink1, plink2; ///< the colliding links if a collision involves a bodies. Collisions do not always occur with 2 bodies like ray collisions, so these fields can be empty.

    //KinBody::Link::GeomConstPtr pgeom1, pgeom2; ///< the specified geometries hit for the given links
    int numCols;            ///< this is the number of objects that collide with the object of interest
    std::vector<KinBody::LinkConstPtr> vLinkColliding; ///< objects colliding with this object

    dReal minDistance;      ///< minimum distance from last query, filled if CO_Distance option is set
    int numWithinTol;       ///< number of objects within tolerance of this object, filled if CO_UseTolerance option is set

    std::vector<CONTACT> contacts; ///< the convention is that the normal will be "out" of plink1's surface. Filled if CO_UseContacts option is set.

    virtual void Reset(int coloptions = 0);
    virtual std::string __str__() const;
};

typedef CollisionReport COLLISIONREPORT;

} // end namespace OpenRAVE

#endif
