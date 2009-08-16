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
#ifndef OPENRAVE_PLANNER_H
#define OPENRAVE_PLANNER_H

namespace OpenRAVE {

/*! \brief Planner interface that generates trajectories for the robot to follow around the environment
 *
 * Planner should be able to query sensor information from the Robot like its current camera image etc.
 * DLL should be pluggable directly into a real hardware platform without any modifications.
 * Planner should be compatible with Robot presented (some hand-shaking happens between the two classes).
 * Examples of planners are:
 * - Manipulation - manipulable objects need to be specified. Objects like doors should be special cases that planners knows about.
 * - Following - Goal easily changes. Attributes can change.
 * - Object Building - Need to describe how parts of object fit together into a bigger part.
 * - Dish Washing - Specific goals are not specified, just a condition that all plates need to be inside.
 * - Foot step planning - Need discrete footsteps and other capabilities from robot.
*/
class PlannerBase : public InterfaceBase
{
public:
    /// evaluator class for a cost function used by planners
    class CostFunction
    {
    public:
        virtual ~CostFunction() {}

        virtual float Eval(const void* pConfiguration) = 0;

        /// output the cost function parameters in a string (usually in XML format)
        virtual bool serialize(std::ostream& O) const { return true; }
    };

    /// evaluator class for a goal function used by planners
    class GoalFunction
    {
    public:
        virtual ~GoalFunction() {}

        virtual float Eval(const void* pConfiguration) = 0;
        virtual float GetGoalThresh() = 0; ///< in goal if Eval() < GetGoalThresh()
        virtual void SetRobot(RobotBase* robot) = 0;

        /// output the cost function parameters in a string (usually in XML format)
        virtual bool serialize(std::ostream& O) const { return true; }
    };

    /// evaluator class for a state space distance metric used by planners
    class DistanceMetric
    {
    public:
        virtual ~DistanceMetric() {}

        /// evaluates two configurations of the robot (usually joint angles)
        virtual float Eval(const void* c0, const void* c1) = 0;
      
        virtual void SetRobot(RobotBase* robot) { _robot = robot; }

        /// output the cost function parameters in a string (usually in XML format)
        virtual bool serialize(std::ostream& O) const { return true; }

        float thresh; ///< if Eval() < thresh, then the two configurations can be considered the same

    protected:
        RobotBase* _robot;
    };

    /// Fills pNewSample with a new sample configuration that is close to pCurSample (pCurSample can be ignored)
    class SampleFunction
    {
    public:
        virtual ~SampleFunction() {}

        /// fills a random configuration. The dimension of pNewSample is usually the dimension the planner is planning in
        virtual void Sample(dReal* pNewSample) = 0;

        /// returns a random configuration around pCurSample
        /// pCurSample - the neighborhood to sample around
        /// fRadius - specifies the max distance of sampling. The higher the value, the farther the samples will go
        ///             The distance metric can be arbitrary, but is usually PlannerParameters::pdistmetric.
        /// \return if sample was successfully generated return true, otherwise false
        virtual bool Sample(dReal* pNewSample, const dReal* pCurSample, dReal fRadius) = 0;

        /// output the cost function parameters in a string (usually in XML format)
        virtual bool serialize(std::ostream& O) const { return true; }
    };

    /// used to set the configuration state and gets its degrees of freedom
    class ConfigurationState
    {
    public:
        virtual ~ConfigurationState() {}

        virtual void SetRobot(RobotBase* probot)=0;
        virtual void SetState(const dReal* pstate)=0;
        virtual void GetState(dReal* pstate)=0;
        virtual void GetLimits(dReal* plower, dReal* pupper)=0;
        
        // helper functions
        virtual void SetState(const std::vector<dReal>& state) { assert((int)state.size()==GetDOF()); SetState(&state[0]); }
        virtual void GetState(std::vector<dReal>& state) { state.resize(GetDOF()); GetState(&state[0]); }
        virtual void GetLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper) {
            vlower.resize(GetDOF()); vupper.resize(GetDOF());
            GetLimits(&vlower[0], &vupper[0]);
        }

        virtual int GetDOF() const = 0;

        /// output the cost function parameters in a string (usually in XML format)
        virtual bool serialize(std::ostream& O) const { return true; }
    };

    enum ConstraintSettings  {
        CS_TimeBackward=1, ///< if not specified, time is forward
    };
    
    /// used to maintain certains a movement from a src robot configuration to robot configuration.
    /// pSrcConf is the configuration the robot is currently at
    /// pDestConf is the configuration the robot should mvoe to
    /// The function returns true if pConf is accepted. Note that the function can also modify
    /// pDestConf (projecting onto a constraint manifold), therefore use the new pDestConf value after this call
    /// ptrans is the transformations of all the links in the robot (can be NULL)
    class ConstraintFunction
    {
    public:
        virtual ~ConstraintFunction() {}
        
        virtual bool Constraint(const dReal* pSrcConf, dReal* pDestConf, Transform* ptrans, int settings) = 0;

        /// output the cost function parameters in a string (usually in XML format)
        virtual bool serialize(std::ostream& O) const { return true; }
    };

    /// Describes a common interface for planning parameters.
    /// If extra parameters need to be specified, 
    /// derive from this class and override the startElement, endElement, and possibly characters calls
    /// The class can be serialized using the <<, >>, and serialize
    class PlannerParameters : public BaseXMLReader
    {
    public:
        PlannerParameters() : pcostfn(NULL), pgoalfn(NULL), pdistmetric(NULL), pconstraintfn(NULL), pSampleFn(NULL), pConfigState(NULL), nMaxIterations(0), bHasWorkspaceGoal(false), _pcurreader(NULL) {}
        PlannerParameters(const PlannerParameters& r);
        virtual ~PlannerParameters() {}

        /// tries to copy data from one set of parameters to another in the safest manner.
        /// First serializes the data of the right hand into a string, then initializes the current parameters struct via >>
        /// pointers to functions are copied directly
        virtual PlannerParameters& operator=(const PlannerParameters& r);
        virtual void copy(const PlannerParameters& r);

        CostFunction* pcostfn;          ///< Cost function on the state pace
        GoalFunction* pgoalfn;          ///< Goal heuristic function
        DistanceMetric* pdistmetric;    ///< Distance metric between configuration spaces

        std::vector<dReal> vinitialconfig, vgoalconfig; ///to specify multiple goal configurations, put them into the vector in series (note: not all planners support multiple goals)

        ConstraintFunction* pconstraintfn; ///< if not null, use it to constrain all new configurations

        SampleFunction* pSampleFn;             ///< use for sampling random states
        ConfigurationState* pConfigState;      ///< used to set/get the configuration state for the planner
                                               ///< by default a planner should use the Active degrees of freedom of the robot

        Transform tWorkspaceGoal;       ///< goal transformation in workspace, can be the end effector or absolute transformation ,etc

        std::vector<float> vParameters; ///< extra float parameters that are specific to the planner (if absolutely necessary)
        std::vector<int>  vnParameters; ///< extra int parameters that are specific to the planner (if absolutely necessary)
        int nMaxIterations;             ///< maximum number of iterations before the planner gives up. If < 0, ignored

        bool bHasWorkspaceGoal;         ///< true if tWorkspaceGoal is valid

    protected:

        /// output the planner parameters in a string (usually in XML format)
        /// don't use PlannerParameters as a tag!
        virtual bool serialize(std::ostream& O) const;
                               
        //@{ XML parsing functions, parses the default parameters
        virtual void* Release() { return this; }
        virtual void startElement(void *ctx, const char *name, const char **atts);
        
        virtual bool endElement(void *ctx, const char *name);
        virtual void characters(void *ctx, const char *ch, int len);
        std::stringstream _ss; ///< holds the data read by characters
        //@}

    private:
        BaseXMLReader* _pcurreader; ///< temporary reader

        /// outputs the data and surrounds it with <PlannerParameters> tags
        friend std::ostream& operator<<(std::ostream& O, const PlannerParameters& v);
        /// expects <PlannerParameters> to be the first token. Parses stream until </PlannerParameters> reached
        friend std::istream& operator>>(std::istream& I, PlannerParameters& v);
    };

    PlannerBase(EnvironmentBase* penv) : InterfaceBase(PT_Planner, penv) {}
    virtual ~PlannerBase() {}

    /// Setup scene, robot, and properties of the plan, and reset all structures with pparams
    /// \param pbase The robot will be planning for.
    /// \param pparams The parameters of the planner, any class derived from PlannerParameters can be passed
    virtual bool InitPlan(RobotBase* pbase, const PlannerParameters* pparams) = 0;

    /// Executes the main planner trying to solve for the goal condition. Fill ptraj with the trajectory
    /// of the planned path that the robot needs to execute
    /// \param ptraj The trajectory the robot has to follow in order to successfully complete the plan
    /// \param pOutStream If specified, planner will output any other special data
    /// \return true if planner is successful
    virtual bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream = NULL) = 0;
    
    /// returns a string describing planner and specification of what values are used in PlannerParameters
    virtual const wchar_t* GetDescription() const = 0;

    virtual RobotBase* GetRobot() const = 0;

private:
    virtual const char* GetHash() const { return OPENRAVE_PLANNER_HASH; }
};

std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v);
std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& v);

} // end namespace OpenRAVE

#endif
