// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file planner.h
    \brief Planning related defintions.
*/
#ifndef OPENRAVE_PLANNER_H
#define OPENRAVE_PLANNER_H

namespace OpenRAVE {

/** \brief Planner interface that generates trajectories for the robot to follow around the environment

    \ingroup interfaces
    Planner should be able to query sensor information from the Robot like its current camera image
    etc. Planner should be compatible with Robot presented (some hand-shaking happens between the two
    classes). Examples of planners are: 
    - Manipulation - manipulable objects need to be specified. Objects like doors should be special cases that planners knows about.
    - Following - Goal easily changes. Attributes can change.
    - Object Building - Need to describe how parts of object fit together into a bigger part.
    - Dish Washing - Specific goals are not specified, just a condition that all plates need to be inside.
    - Foot step planning - Need discrete footsteps and other capabilities from robot.
*/
class RAVE_API PlannerBase : public InterfaceBase
{
public:
    /// Options for constraint planning.
    enum ConstraintSettings  {
        CS_TimeBackward=1, ///< if not specified, time is forward
    };

    /** \brief Describes a common and serializable interface for planning parameters.

        The class is serializable to XML, so can be loaded from file or passed around the network.
        If extra parameters need to be specified, derive from this class and
        - add the extra tags to PlannerParameters::_vXMLParameters
        - override PlannerParameters::startElement and PlannerParameters::endElement for processing
        - possibly override the PlannerParameters::characters
    */
    class RAVE_API PlannerParameters : public BaseXMLReader, public XMLReadable
    {
    public:
        PlannerParameters();
        virtual ~PlannerParameters() {}

        /// tries to copy data from one set of parameters to another in the safest manner.
        /// First serializes the data of the right hand into a string, then initializes the current parameters struct via >>
        /// pointers to functions are copied directly
        virtual PlannerParameters& operator=(const PlannerParameters& r);
        virtual void copy(boost::shared_ptr<PlannerParameters const> r);

        /// sets up the planner parameters to use the active joints of the robot
        void SetRobotActiveJoints(RobotBasePtr robot);

        /// \brief Cost function on the state pace (optional).
        ///
        /// cost = costfn(config)
        /// \param cost the cost of being in the current state
        boost::function<dReal(const std::vector<dReal>&)> _costfn;

        /// \brief Goal heuristic function.
        ///
        /// goal is complete when returns 0 (optional)
        /// distance = goalfn(config)
        /// \param distance - distance to closest goal
        boost::function<dReal(const std::vector<dReal>&)> _goalfn;

        /// optional, Distance metric between configuration spaces, two configurations are considered the same when this returns 0: distmetric(config1,config2)
        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

        /// \brief Filters the current robot configurations (optional).
        ///
        /// optional, used to maintain certains a movement from a src robot configuration to robot configuration:
        /// success = _constraintfn(vprevconf,vnewconf,settings)
        /// When called, vnewconf is guaranteed to be set on the robot. The function returns true if vnewconf is accepted.
        /// Note that the function can also modify vnewconf (like projecting onto a constraint manifold),
        /// therefore the planner will use the new vnewconf value after this call.
        /// \param vprevconf is the configuration the robot is coming from
        /// \param vnewconf is the configuration the robot is current at, which needs to be filtered
        /// \param settings options specified in ConstraingSettings 
        boost::function<bool(const std::vector<dReal>&, std::vector<dReal>&, int)> _constraintfn;

        /// \brief Samples a random configuration (mandatory)
        ///
        /// The dimension of the returned sample is the dimension of the configuration space.
        /// success = samplefn(newsample)
        boost::function<bool(std::vector<dReal>&)> _samplefn;

        /// \brief Samples a valid goal configuration (optional).
        ///
        /// If valid, the function should be called
        /// at every iteration. Any type of goal sampling probabilities and conditions can be encoded inside the function.
        // The dimension of the returned sample is the dimension of the configuration space.
        // success = samplegoalfn(newsample)
        boost::function<bool(std::vector<dReal>&)> _samplegoalfn;

        /// \brief Returns a random configuration around a neighborhood (optional).
        /// 
        /// _sampleneighfn(newsample,pCurSample,fRadius)
        /// \param pCurSample - the neighborhood to sample around
        /// \param  fRadius - specifies the max distance of sampling. The higher the value, the farther the samples will go
        ///             The distance metric can be arbitrary, but is usually PlannerParameters::pdistmetric.
        /// \return if sample was successfully generated return true, otherwise false
        boost::function<bool(std::vector<dReal>&, const std::vector<dReal>&, dReal)> _sampleneighfn;

        /// \brief Sets the state of the robot. Default is active robot joints (mandatory).
        boost::function<void(const std::vector<dReal>&)> _setstatefn;
        /// \brief Gets the state of the robot. Default is active robot joints (mandatory).
        boost::function<void(std::vector<dReal>&)> _getstatefn;
        /// \brief  Computes the difference of two states.
        ///
        /// An explicit difference function is necessary for correct interpolation when there are circular joints.
        /// Default is regular subtraction.
        /// _diffstatefn(q1,q2) -> q1 -= q2
        boost::function<void(std::vector<dReal>&,const std::vector<dReal>&)> _diffstatefn;

        /// to specify multiple goal configurations, put them into the vector in series (note: not all planners support multiple goals)
        std::vector<dReal> vinitialconfig, vgoalconfig;

        /// goal transformation in workspace, can be the end effector or absolute transformation ,etc
        boost::shared_ptr<Transform> _tWorkspaceGoal;

        /// the absolute limits of the configuration space.
        std::vector<dReal> _vConfigLowerLimit,_vConfigUpperLimit;

        /// the discretization resolution of each dimension of the configuration space
        std::vector<dReal> _vConfigResolution;
        
        /// a minimum distance between neighbors when searching. If 0 or less, planner chooses best step length
        dReal _fStepLength;

        /// maximum number of iterations before the planner gives up. If 0 or less, planner chooses best iterations.
        int _nMaxIterations;

        /// specifies the planner that will perform the post-processing path smoothing before returning.
        /// If empty, will not path smooth the returned trajectories (used to measure algorithm time)
        std::string _sPathOptimizationPlanner;
        /// The serialized planner parameters to pass to the path optimizer. For example:
        /// std::stringstream(_sPathOptimizationParameters) >> _parameters;
        std::string _sPathOptimizationParameters;

        /// extra parameters data that does not fit within this planner parameters structure, but is still important not to lose all the information.
        std::string _sExtraParameters;
        
        /// if true, will validate every configuration with a self-collision check. If false, will assume that the configuration
        /// space does not change the self-collision result (ie, planning only with affine DOF)
        /// default is true
        bool _bCheckSelfCollisions;

        /// \return the degrees of freedom of the planning configuration space
        virtual int GetDOF() const { return (int)_vConfigLowerLimit.size(); }

    protected:
        /// output the planner parameters in a string (in XML format)
        /// don't use PlannerParameters as a tag!
        virtual bool serialize(std::ostream& O) const;
                               
        //@{ XML parsing functions, parses the default parameters
        virtual ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts);
        virtual bool endElement(const std::string& name);
        virtual void characters(const std::string& ch);
        std::stringstream _ss; ///< holds the data read by characters
        boost::shared_ptr<std::stringstream> _sslocal;
        /// all the top-level XML parameter tags (lower case) that are handled by this parameter structure, should be registered in the constructor
        std::vector<std::string> _vXMLParameters;
        //@}

    private:
        /// disallow copy constructors since it gets complicated with virtualization
        PlannerParameters(const PlannerParameters& r) : XMLReadable("") { BOOST_ASSERT(0); }
        BaseXMLReaderPtr __pcurreader; ///< temporary reader
        std::string __processingtag;
        int _plannerparametersdepth;

        /// outputs the data and surrounds it with \verbatim <PlannerParameters> \endverbatim tags
        friend RAVE_API std::ostream& operator<<(std::ostream& O, const PlannerParameters& v);
        /// expects \verbatim <PlannerParameters> \endverbatim to be the first token. Parses stream until \verbatim </PlannerParameters> \endverbatim reached
        friend RAVE_API std::istream& operator>>(std::istream& I, PlannerParameters& v);
    };
    typedef boost::shared_ptr<PlannerParameters> PlannerParametersPtr;
    typedef boost::shared_ptr<PlannerParameters const> PlannerParametersConstPtr;

    PlannerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Planner, penv) {}
    virtual ~PlannerBase() {}

    /// \return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_Planner; }
    
    /// \brief Setup scene, robot, and properties of the plan, and reset all internal structures.
    /// \param probot The robot will be planning for.
    /// \param pparams The parameters of the planner, any class derived from PlannerParameters can be passed. The planner should copy these parameters for future instead of storing the pointer.
    virtual bool InitPlan(RobotBasePtr probot, PlannerParametersConstPtr pparams) = 0;

    /// \brief Setup scene, robot, and properties of the plan, and reset all structures with pparams.
    /// \param pbase The robot will be planning for.
    /// \param isParameters The serialized form of the parameters. By default, this exists to allow third parties to
    /// pass information to planners without excplicitly knowning the format/internal structures used
    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters);

    /// \brief Executes the main planner trying to solve for the goal condition.
    ///
    /// Fill ptraj with the trajectory of the planned path that the robot needs to execute
    /// \param ptraj The output trajectory the robot has to follow in order to successfully complete the plan. If this planner is a path optimizer, the trajectory can be used as an input for generating a smoother path. The trajectory is for the configuration degrees of freedom defined by the planner parameters.
    /// \param pOutStream If specified, planner will output any other special data
    /// \return true if planner is successful
    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream = boost::shared_ptr<std::ostream>()) = 0;

    /// \return the internal parameters of the planner
    virtual PlannerParametersConstPtr GetParameters() const = 0;

protected:
    /// Calls a planner to optimizes the trajectory path. The PlannerParameters structure passed into the optimization planner is
    /// constructed with the same freespace constraints as this planner.
    /// This function should always be called in PlanPath to post-process the trajectory.
    /// \param probot the robot this trajectory is meant for, also uses the robot for checking collisions.
    /// \param ptraj Initial trajectory to be smoothed is inputted. If optimization path succeeds, final trajectory output is set in this variable. The trajectory is for the configuration degrees of freedom defined by the planner parameters.
    virtual bool _OptimizePath(RobotBasePtr probot, TrajectoryBasePtr ptraj);
    
private:
    virtual const char* GetHash() const { return OPENRAVE_PLANNER_HASH; }
};

RAVE_API std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v);
RAVE_API std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& v);

} // end namespace OpenRAVE

#endif
