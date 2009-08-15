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
#ifndef OPENRAVE_CONTROLLER_H
#define OPENRAVE_CONTROLLER_H

namespace OpenRAVE {

/// Abstract base class to encapsulate a local controller. All commands
/// given to the robot are first filtered through here, then translated
/// to joint commands. The default controller takes angles. Different controllers
/// have different path inputs (ie: a robot walking on a floor might just have x,y,angle)
class ControllerBase : public InterfaceBase
{
public:
    enum ActuatorState {
        AS_Undefined=0, ///< returned when no state is defined
        AS_Idle=1,  ///< this actuator is idle
        AS_Moving=2, ///< this actuator is in motion from previous commands
        AS_Stalled=3, ///< the actuator is stalled, needs to be unstalled by sending a ready signal
        AS_Braked=4, ///< the actuator is braked
    };

    ControllerBase(EnvironmentBase* penv) : InterfaceBase(penv) {}
    virtual ~ControllerBase() {}
    
    /// Initializes the controller
    /// \param robot the robot that uses the controller
    /// \param args extra arguments that the controller takes.
    /// \return true on successful initialization
    virtual bool Init(RobotBase* robot, const char* args = NULL) = 0;

    /// Resets the current controller trajectories and any other state associated with the robot
    /// \param options - specific options that can be used to control what to reset
    virtual void Reset(int options) = 0;

    /// go to a specific position in configuration space
    /// \param pValues - final configuration
    /// \return true if position operation successful
    virtual bool SetDesired(const dReal* pValues) = 0;
    
    /// follow a path in configuration space, adds to the queue of trajectories already in execution
    /// \param ptraj - the trajectory
    /// \return true if trajectory operation successful
    virtual bool SetPath(const Trajectory* ptraj) = 0;

    /// replace a previous trajectory that has been sent for execution, 
    /// \param ptraj - the trajectory to be inserted
    /// \param nTrajectoryId - the unique id of this trajectory. If it is similar to previous trajectory ids
    /// the controller will attempt to replace the queued trajectory with the current trajectory.
    /// \param fDivergenceTime - the time where this new trajectory diverges from an old
    /// trajectory with a similar nTrajectoryId
    /// \return true if trajectory insertion successful
    virtual bool SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime) = 0;
    
    /// Simulate one step forward for controllers running in the simulation environment
    /// \param fTimeElapsed - time elapsed in simulation environment since last frame
    /// \return true if the controller is done with the current commands it is given
    virtual bool SimulationStep(dReal fTimeElapsed) = 0;

    /// \return true when goal reached. If a trajectory was set, return only when
    ///         trajectory is done. If SetDesired was called, return only when robot is
    ///         is at the desired location. If SendCmd sent, returns true when the command
    ///         was completed by the hand.
    virtual bool IsDone() = 0;

    /// \return the time along the current command
    virtual float GetTime() const = 0;

    /// get velocity
    /// \param vel [out] - current velocity of robot
    virtual void GetVelocity(std::vector<dReal>& vel) const {}

    /// get torque/current/strain values
    /// \param torque [out] - returns the current torque/current/strain exerted by each of the joints from outside forces.
    /// The feedforward and friction terms should be subtracted out already
    virtual void GetTorque(std::vector<dReal>& torque) const {}
    
    virtual RobotBase* GetRobot() const = 0;

    /// \return a state corresponding to the actuator of the given index
    virtual ActuatorState GetActuatorState(int index) const {return AS_Undefined;}

    /// Used to send special commands to the controller
    /// \param pcmd the first word is the command name, the following text is the parameters
    /// \return true if the command is successfully processed by the controller
    virtual bool SendCmd(const char* pcmd) { return false; }

    /// \return true if the controller supports the specified command
    virtual bool SupportsCmd(const char* pcmd) { return false; }

private:
    virtual const char* GetHash() const { return OPENRAVE_CONTROLLER_HASH; }
};

} // end namespace OpenRAVE

#endif
