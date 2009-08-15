// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

#include <boost/shared_ptr.hpp>

/// A generic robot
class GenericRobot : public RobotBase
{
public:
    enum RobotState { ST_NONE=0, ST_PD_CONTROL, ST_PATH_FOLLOW };
    //ST_SINGLE_MOTION, ST_REACH,
	//	    ST_GRAB,   ST_TRANSFER,    ST_LOOP_MOTION,   ST_RELEASE,
	//	    ST_RETURN, ST_DYN_SIM,     NUM_STATES };

    GenericRobot(EnvironmentBase* penv);
    virtual ~GenericRobot();

    virtual void Destroy();

    virtual const char* GetXMLId() { return "GenericRobot"; }

#ifdef RAVE_GUIBUILD
    /** @name Visual Aids
    * Markers and visual aids used for GUI and rendering */
    //@{
    void SetPathVisibility(bool bFlag);
    void SetTrackingVisibility(bool bFlag);
    void SetIKHandleVisibility(bool bFlag);
    void SetViewFrustumVisibility(bool bFlag);
    void SetCentroidVisibility(bool bFlag);
    void SetSkeletonVisibility(bool bFlag);
    //@}
#endif

    virtual void SetMotion(const Trajectory* ptraj);
    virtual bool SetController(const wchar_t* pname, const char* args=NULL, bool bDestroyOldController=true);
    virtual bool SetController(ControllerBase * p, const char* args=NULL, bool bDestroyOldController=true);

    virtual void SetActiveMotion(const Trajectory* ptraj);

    RobotState GetState() { return _state; }

    virtual ControllerBase* GetController() const { return _pController; }

    virtual void SimulationStep(dReal fElapsedTime);

protected:

    // manipulators
    ControllerBase* _pController;       ///< follows a trajectory
    boost::shared_ptr<Trajectory> _curTrajectory;

    RobotState _state;

    // markers and visual aids
    bool        _bDisplayTrack;
    bool        _bDisplayCentroid;
    bool        _bUseController;

    friend class RobotXMLReader;
};
