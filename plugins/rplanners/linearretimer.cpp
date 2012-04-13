// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "trajectoryretimer.h"

class LinearTrajectoryRetimer : public TrajectoryRetimer
{
public:
    LinearTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryRetimer(penv,sinput)
    {
        __description = ":Interface Author: Rosen Diankov\n\nLinear trajectory re-timing without modifying any of the points. Overwrites the velocities and timestamps.";
    }

protected:
    virtual bool _SupportInterpolation() {
        if( _parameters->_interpolation.size() == 0 ) {
            _parameters->_interpolation = "linear";
            return true;
        }
        else {
            return _parameters->_interpolation == "linear";
        }
    }

    dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        dReal bestmintime = 0;
        if( info->orgveloffset >= 0 ) {
            for(int i = 0; i < info->gpos.dof; ++i) {
                dReal mintime = RaveFabs(*(itorgdiff+info->orgposoffset+i) / *(itorgdiff+info->orgveloffset+i));
                bestmintime = max(bestmintime,mintime);
            }
        }
        else {
            for(int i = 0; i < info->gpos.dof; ++i) {
                dReal mintime = RaveFabs(*(itorgdiff+info->orgposoffset+i)*_vimaxvel.at(info->orgposoffset+i));
                bestmintime = max(bestmintime,mintime);
            }
        }
        return bestmintime;
    }

    void _ComputeVelocitiesJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        if( *(itdata+_timeoffset) > 0 ) {
            dReal invdeltatime = 1.0 / *(itdata+_timeoffset);
            for(int i = 0; i < info->gpos.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itorgdiff+info->orgposoffset+i)*invdeltatime;
            }
        }
        else {
            // copy the velocity?
            for(int i = 0; i < info->gpos.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itdataprev+info->gvel.offset+i);
            }
        }
    }

    dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        dReal bestmintime = 0;
        const boost::array<DOFAffine,4> testdofs={{DOF_X,DOF_Y,DOF_Z,DOF_RotationAxis}};
        dReal distxyz = 0;
        dReal fivel=0;
        FOREACHC(itdof,testdofs) {
            if( affinedofs & *itdof ) {
                int index = RaveGetIndexFromAffineDOF(affinedofs,*itdof);
                dReal f = *(itorgdiff+index);
                distxyz += f*f;
                fivel = _vimaxvel.at(info->orgposoffset+index);
            }
        }
        if( distxyz > 0 ) {
            dReal mintime = RaveSqrt(distxyz)*fivel;
            bestmintime = max(bestmintime,mintime);
        }
        if( affinedofs & DOF_RotationAxis ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationAxis);
            dReal mintime = RaveFabs(*(itorgdiff+index)*_vimaxvel.at(info->orgposoffset+index));
            bestmintime = max(bestmintime,mintime);
        }
        else if( affinedofs & DOF_RotationQuat ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationQuat);
            Vector qprev,qnext;
            for(int i = 0; i < 4; ++i) {
                qnext[i] = *(itdata+info->gpos.offset+index+i);
                qprev[i] = *(itdataprev+info->gpos.offset+index+i);
            }
            dReal mintime = 2.0*RaveAcos(min(dReal(1),RaveFabs(qprev.dot(qnext))))*_vimaxvel.at(info->orgposoffset+index);
            bestmintime = max(bestmintime,mintime);
        }
        else if( affinedofs & DOF_Rotation3D ) {
            RAVELOG_WARN("_ComputeMinimumTimeAffine does not support DOF_Rotation3D\n");
        }
        return bestmintime;
    }

    void _ComputeVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        if( *(itdata+_timeoffset) > 0 ) {
            dReal invdeltatime = 1.0 / *(itdata+_timeoffset);
            const boost::array<DOFAffine,4> testdofs={{DOF_X,DOF_Y,DOF_Z,DOF_RotationAxis}};
            FOREACHC(itdof,testdofs) {
                if( affinedofs & *itdof ) {
                    int index = RaveGetIndexFromAffineDOF(affinedofs,*itdof);
                    *(itdata+info->gvel.offset+index) = *(itorgdiff+info->orgposoffset+index)*invdeltatime;
                }
            }
            if( affinedofs & DOF_RotationQuat ) {
                int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationQuat);
                for(int i = 0; i < 4; ++i) {
                    *(itdata+info->gvel.offset+index+i) = *(itorgdiff+index+i)*invdeltatime;
                }
            }
            else if( affinedofs & DOF_Rotation3D ) {
                RAVELOG_WARN("_ComputeMinimumTimeAffine does not support DOF_Rotation3D\n");
            }
        }
        else {
            // copy the velocity?
            for(int i = 0; i < info->gpos.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itdataprev+info->gvel.offset+i);
            }
        }
    }

    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        IkParameterization ikparamprev, ikparam;
        ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
        ikparam.Set(itdata+info->gpos.offset,iktype);
        switch(iktype) {
        case IKP_Transform6D: {
            dReal quatmintime = 2.0*RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTransform6D().rot.dot(ikparam.GetTransform6D().rot))))*_vimaxvel.at(info->orgposoffset+0);
            dReal transmintime = RaveSqrt((ikparamprev.GetTransform6D().trans-ikparam.GetTransform6D().trans).lengthsqr3())*_vimaxvel.at(info->orgposoffset+4);
            return max(quatmintime,transmintime);
        }
        case IKP_Rotation3D:
            return 2.0*RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetRotation3D().dot(ikparam.GetRotation3D()))))*_vimaxvel.at(info->orgposoffset+0);
        case IKP_Translation3D:
            return RaveSqrt((ikparamprev.GetTranslation3D()-ikparam.GetTranslation3D()).lengthsqr3())*_vimaxvel.at(info->orgposoffset);
        case IKP_Direction3D: {
            return RaveAcos(min(dReal(1),ikparamprev.GetDirection3D().dot3(ikparam.GetDirection3D())))*_vimaxvel.at(info->orgposoffset);
        }
        case IKP_Ray4D: {
            Vector pos0 = ikparamprev.GetRay4D().pos - ikparamprev.GetRay4D().dir*ikparamprev.GetRay4D().dir.dot(ikparamprev.GetRay4D().pos);
            Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
            dReal fcos = ikparamprev.GetRay4D().dir.dot(ikparam.GetRay4D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return max(facos*_vimaxvel.at(info->orgposoffset),(pos0-pos1).lengthsqr3()*_vimaxvel.at(info->orgposoffset+3));
        }
        case IKP_Lookat3D:
            return RaveSqrt((ikparamprev.GetLookat3D()-ikparam.GetLookat3D()).lengthsqr3())*_vimaxvel.at(info->orgposoffset);
        case IKP_TranslationDirection5D: {
            dReal dirmintime = RaveAcos(min(dReal(1),ikparamprev.GetTranslationDirection5D().dir.dot3(ikparam.GetTranslationDirection5D().dir)))*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparamprev.GetTranslationDirection5D().pos-ikparam.GetTranslationDirection5D().pos).lengthsqr3())*_vimaxvel.at(info->orgposoffset+3);
            return max(dirmintime,transmintime);
        }
        case IKP_TranslationXY2D:
            return RaveSqrt((ikparamprev.GetTranslationXY2D()-ikparam.GetTranslationXY2D()).lengthsqr2())*_vimaxvel.at(info->orgposoffset+0);
        case IKP_TranslationXYOrientation3D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationXYOrientation3D().z,ikparamprev.GetTranslationXYOrientation3D().z)*_vimaxvel.at(info->orgposoffset+2);
            dReal transmintime = RaveSqrt((ikparamprev.GetTranslationXYOrientation3D()-ikparam.GetTranslationXYOrientation3D()).lengthsqr2())*_vimaxvel.at(info->orgposoffset+0);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationLocalGlobal6D: {
            dReal transmintime0 = RaveSqrt((ikparamprev.GetTranslationLocalGlobal6D().first-ikparam.GetTranslationLocalGlobal6D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+0);
            dReal transmintime1 = RaveSqrt((ikparamprev.GetTranslationLocalGlobal6D().second-ikparam.GetTranslationLocalGlobal6D().second).lengthsqr3())*_vimaxvel.at(info->orgposoffset+3);
            return max(transmintime0,transmintime1);
        }
        case IKP_TranslationXAxisAngle4D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationXAxisAngle4D().second,ikparamprev.GetTranslationXAxisAngle4D().second)*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparam.GetTranslationXAxisAngle4D().first-ikparamprev.GetTranslationXAxisAngle4D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+1);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationYAxisAngle4D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationYAxisAngle4D().second,ikparamprev.GetTranslationYAxisAngle4D().second)*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparam.GetTranslationYAxisAngle4D().first-ikparamprev.GetTranslationYAxisAngle4D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+1);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationZAxisAngle4D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationZAxisAngle4D().second,ikparamprev.GetTranslationZAxisAngle4D().second)*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparam.GetTranslationZAxisAngle4D().first-ikparamprev.GetTranslationZAxisAngle4D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+1);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationXAxisAngleZNorm4D().second,ikparamprev.GetTranslationXAxisAngleZNorm4D().second)*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparam.GetTranslationXAxisAngleZNorm4D().first-ikparamprev.GetTranslationXAxisAngleZNorm4D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+1);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationYAxisAngleXNorm4D().second,ikparamprev.GetTranslationYAxisAngleXNorm4D().second)*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparam.GetTranslationYAxisAngleXNorm4D().first-ikparamprev.GetTranslationYAxisAngleXNorm4D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+1);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            dReal angmintime = utils::SubtractCircularAngle(ikparam.GetTranslationZAxisAngleYNorm4D().second,ikparamprev.GetTranslationZAxisAngleYNorm4D().second)*_vimaxvel.at(info->orgposoffset);
            dReal transmintime = RaveSqrt((ikparam.GetTranslationZAxisAngleYNorm4D().first-ikparamprev.GetTranslationZAxisAngleYNorm4D().first).lengthsqr3())*_vimaxvel.at(info->orgposoffset+1);
            return max(angmintime,transmintime);
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
        }
    }

    void _ComputeVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        if( *(itdata+_timeoffset) > 0 ) {
            dReal invdeltatime = 1.0 / *(itdata+_timeoffset);
            // can probably do better...
            for(int i = 0; i < info->gpos.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itorgdiff+info->orgposoffset+i)*invdeltatime;
            }
        }
        else {
            // copy the velocity?
            for(int i = 0; i < info->gpos.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itdataprev+info->gvel.offset+i);
            }
        }
    }
};


PlannerBasePtr CreateLinearTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new LinearTrajectoryRetimer(penv, sinput));
}
