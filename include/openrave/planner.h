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
/** \file planner.h
    \brief Planning related defintions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_PLANNER_H
#define OPENRAVE_PLANNER_H

namespace OpenRAVE {

/// \brief the status of the PlanPath method. Used when PlanPath can be called multiple times to resume planning.
enum PlannerStatus
{
    PS_Failed = 0, ///< planner failed
    PS_HasSolution = 1, ///< planner succeeded
    PS_Interrupted = 2, ///< planning was interrupted, but can be resumed by calling PlanPath again
    PS_InterruptedWithSolution = 3, /// planning was interrupted, but a valid path/solution was returned. Can call PlanPath again to refine results
};

/// \brief action to send to the planner while it is planning. This is usually done by the user-specified planner callback function
enum PlannerAction
{
    PA_None=0, ///< no action
    PA_Interrupt=1, ///< interrupt the planner and return to user
    PA_ReturnWithAnySolution=2, ///< return quickly with any path
};

/** \brief <b>[interface]</b> Planner interface that generates trajectories for target objects to follow through the environment. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_planner.
    \ingroup interfaces
 */
class OPENRAVE_API PlannerBase : public InterfaceBase
{
public:
    typedef std::list< std::vector<dReal> > ConfigurationList;
    typedef boost::shared_ptr< PlannerBase::ConfigurationList > ConfigurationListPtr;

    /** \brief Describes a common and serializable interface for planning parameters.

        The class is serializable to XML, so can be loaded from file or passed around the network.
        If extra parameters need to be specified, derive from this class and
        - add the extra tags to PlannerParameters::_vXMLParameters
        - override PlannerParameters::startElement and PlannerParameters::endElement for processing
        - possibly override the PlannerParameters::characters

        Also allows the parameters and descriptions to be serialized to reStructuredText for documentation purposes.
     */
    class OPENRAVE_API PlannerParameters : public BaseXMLReader, public XMLReadable
    {
public:
        PlannerParameters();
        virtual ~PlannerParameters() {
        }

        /// \brief saves and restores the state using PlannerParameters::_setstatefn and PlannerParameters::_getstatefn
        class OPENRAVE_API StateSaver
        {
public:
            StateSaver(boost::shared_ptr<PlannerParameters> params);
            virtual ~StateSaver();
            inline boost::shared_ptr<PlannerParameters> GetParameters() const {
                return _params;
            }
            virtual void Restore();
protected:
            boost::shared_ptr<PlannerParameters> _params;
            std::vector<dReal> _values;
private:
            virtual void _Restore();
        };

        typedef boost::shared_ptr<StateSaver> StateSaverPtr;

        /** \brief Attemps to copy data from one set of parameters to another in the safest manner.

            First serializes the data of the right hand into a string, then initializes the current parameters via >>
            pointers to functions are copied directly
         */
        virtual PlannerParameters& operator=(const PlannerParameters& r);
        virtual void copy(boost::shared_ptr<PlannerParameters const> r);

        /// \brief sets up the planner parameters to use the active joints of the robot
        virtual void SetRobotActiveJoints(RobotBasePtr robot);

        /** \brief sets up the planner parameters to use the configuration specification space

            The configuraiton groups should point to controllable target objects. By default, this includes:
            - joint_values
            - joint_velocities
            - affine_transform
            - affine_velocities
            - grab
            The following internal parameters will be set:
            - _diffstatefn
            - _distmetricfn - weights used for distance metric are retrieved at this time and stored
            - _samplefn
            - _sampleneighfn
            - _setstatefn
            - _getstatefn
            - _neighstatefn
            - _checkpathconstraintsfn
            - _vConfigLowerLimit
            - _vConfigUpperLimit
            - _vConfigVelocityLimit
            - _vConfigAccelerationLimit
            - _vConfigResolution
            - vinitialconfig
            - _configurationspecification
            \throw openrave_exception If the configuration specification is invalid or points to targets that are not present in the environment.
         */
        virtual void SetConfigurationSpecification(EnvironmentBasePtr env, const ConfigurationSpecification& spec);

        /// \brief veriries that the configuration space and all parameters are consistent
        ///
        /// Assumes at minimum that  _setstatefn and _getstatefn are set. Correct environment should be
        /// locked when this function is called since _getstatefn will be called.
        /// \throw openrave_exception If not consistent, will throw an exception
        virtual void Validate() const;

        /// \brief the configuration specification in which the planner works in. This specification is passed to the trajecotry creation modules.
        ConfigurationSpecification _configurationspecification;

        /// \brief Cost function on the state pace (optional).
        ///
        /// cost = _costfn(config)
        /// \param cost the cost of being in the current state
        typedef boost::function<dReal(const std::vector<dReal>&)> CostFn;
        CostFn _costfn;

        /** \brief Goal heuristic function.(optional)

            distance = _goalfn(config)

            Goal is complete when returns 0
            \param distance - distance to closest goal
         */
        typedef boost::function<dReal(const std::vector<dReal>&)> GoalFn;
        GoalFn _goalfn;

        /// \brief Distance metric between configuration spaces (optional)
        ///
        /// distmetric(config1,config2)
        ///
        /// Two configurations are considered the same when function returns 0.
        typedef boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> DistMetricFn;
        DistMetricFn _distmetricfn;

        /** \brief Checks that all the constraints are satisfied between two configurations.

            The simplest and most fundamental constraint is line-collision checking. The robot goes from q0 to q1.

            success = _checkpathconstraints(q0,q1,interval,configurations)

            When called, q0 is guaranteed to be set on the robot.
            The function returns true if the path to q1 satisfies all the constraints of the planner.
            If q0==q1, and interval==IT_OpenStart or IT_OpenEnd, then only one configuration should be checked. It is recommended to use IT_OpenStart.
            Because this function can internally use neighstatefn, need to make sure that Q0->Q1 is going from initial to goal direction.

            \param q0 is the configuration the robot is coming from (currently set).
            \param q1 is the configuration the robot should move to.
            \param interval Specifies whether to check the end points of the interval for constraints
            \param configurations Optional argument that will hold the intermediate configuraitons checked between q0 and q1 configurations. The appended configurations will be all valid and in free space. They are appended after the items already stored on the list.
         */
        typedef boost::function<bool (const std::vector<dReal>&, const std::vector<dReal>&, IntervalType, PlannerBase::ConfigurationListPtr)> CheckPathConstraintFn;
        CheckPathConstraintFn _checkpathconstraintsfn;

        /** \brief Samples a random configuration (mandatory)

            The dimension of the returned sample is the dimension of the configuration space.
            success = samplefn(newsample)
         */
        typedef boost::function<bool (std::vector<dReal>&)> SampleFn;
        SampleFn _samplefn;

        /** \brief Samples a valid goal configuration (optional).

            If valid, the function should be called
            at every iteration. Any type of sampling probabilities and conditions can be encoded inside the function.
            The dimension of the returned sample is the dimension of the configuration space.
            success = samplegoalfn(newsample)
         */
        typedef boost::function<bool (std::vector<dReal>&)> SampleGoalFn;
        SampleGoalFn _samplegoalfn;

        /** \brief Samples a valid initial configuration (optional).

            If valid, the function should be called
            at every iteration. Any type of sampling probabilities and conditions can be encoded inside the function.
            The dimension of the returned sample is the dimension of the configuration space.
            success = sampleinitialfn(newsample)
         */
        typedef boost::function<bool (std::vector<dReal>&)> SampleInitialFn;
        SampleInitialFn _sampleinitialfn;

        /** \brief Returns a random configuration around a neighborhood (optional).

            _sampleneighfn(newsample,pCurSample,fRadius)

            \param pCurSample - the neighborhood to sample around
            \param  fRadius - specifies the max distance of sampling. The higher the value, the farther the samples will go
                              The distance metric can be arbitrary, but is usually PlannerParameters::pdistmetric.
            \return if sample was successfully generated return true, otherwise false
         */
        typedef boost::function<bool (std::vector<dReal>&, const std::vector<dReal>&, dReal)> SampleNeighFn;
        SampleNeighFn _sampleneighfn;

        /// \brief Sets the state of the robot. Default is active robot joints (mandatory).
        typedef boost::function<void (const std::vector<dReal>&)> SetStateFn;
        SetStateFn _setstatefn;
        /// \brief Gets the state of the robot. Default is active robot joints (mandatory).
        typedef boost::function<void (std::vector<dReal>&)> GetStateFn;
        GetStateFn _getstatefn;

        /** \brief  Computes the difference of two states.

            _diffstatefn(q1,q2) -> q1 -= q2

            An explicit difference function is necessary for correct interpolation when there are circular joints.
            Default is regular subtraction.
         */
        typedef boost::function<void (std::vector<dReal>&,const std::vector<dReal>&)> DiffStateFn;
        DiffStateFn _diffstatefn;

        /** \brief Adds a delta state to a curent state, acting like a next-nearest-neighbor function along a given direction.

            success = _neighstatefn(q,qdelta,fromgoal) -> q = Filter(q+qdelta)
            \param q the current state. In order to save computation, assumes this state is the currently set configuration.
            \param qdelta the delta to add
            \param fromgoal 1 if q is coming from a goal state, 0 if it is coming from an initial state

            In RRTs this is used for the extension operation. The new state is stored in the first parameter q.
            Note that the function can also add a filter to the final destination (like projecting onto a constraint manifold).
         */
        typedef boost::function<bool (std::vector<dReal>&,const std::vector<dReal>&, int)> NeighStateFn;
        NeighStateFn _neighstatefn;

        /// to specify multiple initial or goal configurations, put them into the vector in series
        /// size always has to be a multiple of GetDOF()
        /// note: not all planners support multiple goals
        std::vector<dReal> vinitialconfig, vgoalconfig;

        /// \brief the absolute limits of the configuration space.
        std::vector<dReal> _vConfigLowerLimit, _vConfigUpperLimit;

        /// \brief the absolute velocity limits of each DOF of the configuration space.
        std::vector<dReal> _vConfigVelocityLimit;

        /// \brief the absolute acceleration limits of each DOF of the configuration space.
        std::vector<dReal> _vConfigAccelerationLimit;

        /// \brief the discretization resolution of each dimension of the configuration space
        std::vector<dReal> _vConfigResolution;

        /** \brief a discretization between the path that connects two configurations

            This length represents how dense the samples get distributed across the configuration space.
            It represents the maximum distance between neighbors when adding new configuraitons.
            If 0 or less, planner chooses best step length.
         */
        dReal _fStepLength;

        /// \brief maximum number of iterations before the planner gives up. If 0 or less, planner chooses best iterations.
        int _nMaxIterations;

        /// \brief Specifies the planner that will perform the post-processing path smoothing before returning.
        ///
        /// If empty, will not path smooth the returned trajectories (used to measure algorithm time)
        std::string _sPostProcessingPlanner;

        /// \brief The serialized planner parameters to pass to the path optimizer.
        ///
        /// For example: std::stringstream(_sPostProcessingParameters) >> _parameters;
        std::string _sPostProcessingParameters;

        /// \brief Extra parameters data that does not fit within this planner parameters structure, but is still important not to lose all the information.
        std::string _sExtraParameters;

        /// \brief Return the degrees of freedom of the planning configuration space
        virtual int GetDOF() const {
            return _configurationspecification.GetDOF();
        }

protected:
        inline boost::shared_ptr<PlannerBase::PlannerParameters> shared_parameters() {
            return boost::static_pointer_cast<PlannerBase::PlannerParameters>(shared_from_this());
        }
        inline boost::shared_ptr<PlannerBase::PlannerParameters const > shared_parameters_const() const {
            return boost::static_pointer_cast<PlannerBase::PlannerParameters const>(shared_from_this());
        }

        /// \brief output the planner parameters in a string (in XML format)
        ///
        /// \param options if 1 will skip writing the extra parameters
        /// don't use PlannerParameters as a tag!
        virtual bool serialize(std::ostream& O, int options=0) const;

        //@{ XML parsing functions, parses the default parameters
        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
        virtual bool endElement(const std::string& name);
        virtual void characters(const std::string& ch);
        std::stringstream _ss;         ///< holds the data read by characters
        boost::shared_ptr<std::stringstream> _sslocal;
        /// all the top-level XML parameter tags (lower case) that are handled by this parameter structure, should be registered in the constructor
        std::vector<std::string> _vXMLParameters;
        //@}

private:
        /// prevent copy constructors since it gets complicated with virtual functions
        PlannerParameters(const PlannerParameters &r);
        BaseXMLReaderPtr __pcurreader;         ///< temporary reader
        std::string __processingtag;
        int _plannerparametersdepth;

        /// outputs the data and surrounds it with \verbatim <PlannerParameters> \endverbatim tags
        friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const PlannerParameters& v);
        /// expects \verbatim <PlannerParameters> \endverbatim to be the first token. Parses stream until \verbatim </PlannerParameters> \endverbatim reached
        friend OPENRAVE_API std::istream& operator>>(std::istream& I, PlannerParameters& v);
    };
    typedef boost::shared_ptr<PlannerBase::PlannerParameters> PlannerParametersPtr;
    typedef boost::shared_ptr<PlannerBase::PlannerParameters const> PlannerParametersConstPtr;
    typedef boost::weak_ptr<PlannerBase::PlannerParameters> PlannerParametersWeakPtr;

    /// \brief Planner progress information passed to each callback function
    class OPENRAVE_API PlannerProgress
    {
public:
        PlannerProgress();
        int _iteration;
    };

    PlannerBase(EnvironmentBasePtr penv);
    virtual ~PlannerBase() {
    }

    /// \return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_Planner;
    }

    /** \brief Setup scene, robot, and properties of the plan, and reset all internal structures.

        \param robot main robot to be used for planning
        \param params The parameters of the planner, any class derived from PlannerParameters can be passed. The planner should copy these parameters for future instead of storing the pointer.
     */
    virtual bool InitPlan(RobotBasePtr robot, PlannerParametersConstPtr params) = 0;

    /** \brief Setup scene, robot, and properties of the plan, and reset all structures with pparams.

        \param robot main robot to be used for planning
        \param isParameters The serialized form of the parameters. By default, this exists to allow third parties to
        pass information to planners without excplicitly knowning the format/internal structures used
        \return true if plan is initialized successfully and initial conditions are satisfied.
     */
    virtual bool InitPlan(RobotBasePtr robot, std::istream& isParameters);

    /** \brief Executes the main planner trying to solve for the goal condition.

        Fill ptraj with the trajectory of the planned path that the robot needs to execute
        \param ptraj The output trajectory the robot has to follow in order to successfully complete the plan. If this planner is a path optimizer, the trajectory can be used as an input for generating a smoother path. The trajectory is for the configuration degrees of freedom defined by the planner parameters.
        \return the status that the planner returned in.
     */
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj) = 0;

    /// \deprecated (11/10/03)
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream) RAVE_DEPRECATED {
        if( !!pOutStream ) {
            RAVELOG_WARN("planner does not support pOutputStream anymore, please find another method to return information like using SendCommand or writing the data into the returned trajectory\n");
        }
        return PlanPath(ptraj);
    }

    /// \brief return the internal parameters of the planner
    virtual PlannerParametersConstPtr GetParameters() const = 0;

    /** \brief Callback function during planner execute

        \param progress planner progress information
     */
    typedef boost::function<PlannerAction(const PlannerProgress&)> PlanCallbackFn;

    /** \brief register a function that is called periodically during the plan loop.

        Allows the calling process to control the behavior of the planner from a high-level perspective
     */
    virtual UserDataPtr RegisterPlanCallback(const PlanCallbackFn& callbackfn);

protected:
    inline PlannerBasePtr shared_planner() {
        return boost::static_pointer_cast<PlannerBase>(shared_from_this());
    }
    inline PlannerBaseConstPtr shared_planner_const() const {
        return boost::static_pointer_cast<PlannerBase const>(shared_from_this());
    }

    /** \brief Calls a planner to optimizes the trajectory path.

        The PlannerParameters structure passed into the optimization planner is
        constructed with the same freespace constraints as this planner.
        This function should always be called in PlanPath to post-process the trajectory.
        \param probot the robot this trajectory is meant for, also uses the robot for checking collisions.
        \param ptraj Initial trajectory to be smoothed is inputted. If optimization path succeeds, final trajectory output is set in this variable. The trajectory is for the configuration degrees of freedom defined by the planner parameters.
     */
    virtual PlannerStatus _ProcessPostPlanners(RobotBasePtr probot, TrajectoryBasePtr ptraj);

    virtual bool _OptimizePath(RobotBasePtr probot, TrajectoryBasePtr ptraj) RAVE_DEPRECATED {
        return !!(_ProcessPostPlanners(probot,ptraj) & PS_HasSolution);
    }

    /// \brief Calls the registered callbacks in order and returns immediately when an action other than PA_None is returned.
    ///
    /// \param progress planner progress information
    virtual PlannerAction _CallCallbacks(const PlannerProgress& progress);

private:
    virtual const char* GetHash() const {
        return OPENRAVE_PLANNER_HASH;
    }

    std::list<UserDataWeakPtr> __listRegisteredCallbacks; ///< internally managed callbacks

    friend class CustomPlannerCallbackData;
};

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v);
OPENRAVE_API std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& v);

} // end namespace OpenRAVE

#endif
