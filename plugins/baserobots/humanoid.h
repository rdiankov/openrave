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

/// A generic robot
class Humanoid : public RobotBase
{
public:
    enum RobotState { ST_NONE=0, ST_PD_CONTROL, ST_PATH_FOLLOW };
    //ST_SINGLE_MOTION, ST_REACH,
	//	    ST_GRAB,   ST_TRANSFER,    ST_LOOP_MOTION,   ST_RELEASE,
	//	    ST_RETURN, ST_DYN_SIM,     NUM_STATES };

    Humanoid(EnvironmentBase* penv);
    virtual ~Humanoid();

    virtual void Destroy();

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

    virtual const char* GetXMLId() { return "Humanoid"; }

    virtual void SetMotion(const Trajectory* ptraj);
    virtual void SetActiveMotion(const Trajectory* ptraj);

    virtual bool SetController(const wchar_t* pname, const char* args=NULL, bool bDestroyOldController=true);
    virtual bool SetController(ControllerBase * p, const char* args=NULL, bool bDestroyOldController=true);

    RobotState GetState() { return _state; }

    virtual void SetUseController(bool bUseController) { _bUseController = bUseController; }
    virtual bool GetUseController() const { return _bUseController; }

    virtual ControllerBase* GetController() const { return _pController; }

    virtual void SimulationStep(dReal fElapsedTime);

    virtual bool Init(char* strData, const char**atts);

    // inits local variables after robot is initialized
    virtual bool LocalInit();

    KinBody::Link* _plinkFeet[2]; ///< [0] - left, [1] - right
    KinBody::Link* _plinkHands[2]; ///< [0] - left, [1] - right
    KinBody::Link* _plinkHead;

    // front facing dir with respect to base
    Vector GetFront() const {
        TransformMatrix t = _veclinks.front()->GetTransform();
        Vector front;
        front.x = t.m[0] * _vfront.x + t.m[1] * _vfront.y + t.m[2] * _vfront.z;
        front.y = t.m[4] * _vfront.x + t.m[5] * _vfront.y + t.m[6] * _vfront.z;
        front.z = t.m[8] * _vfront.x + t.m[9] * _vfront.y + t.m[10] * _vfront.z;
        return front;
    }

    // up facing dir with respect to base
    Vector GetUp() const {
        TransformMatrix t = _veclinks.front()->GetTransform();
        Vector up;
        up.x = t.m[0] * _vup.x + t.m[1] * _vup.y + t.m[2] * _vup.z;
        up.y = t.m[4] * _vup.x + t.m[5] * _vup.y + t.m[6] * _vup.z;
        up.z = t.m[8] * _vup.x + t.m[9] * _vup.y + t.m[10] * _vup.z;
        return up;
    }

protected:

    Vector _vfront; // direction of front facing
    Vector _vup;    // direction of head

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
