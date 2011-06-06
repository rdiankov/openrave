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

/** \brief <b>[interface]</b> Planner interface that generates trajectories for the robot to follow around the environment. See \ref arch_planner.
    \ingroup interfaces
*/
class OPENRAVE_API PlannerBase : public InterfaceBase
{
public:
    typedef std::list< std::vector<dReal> > ConfigurationList;
    typedef boost::shared_ptr< ConfigurationList > ConfigurationListPtr;

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
        virtual ~PlannerParameters() {}

        /// tries to copy data from one set of parameters to another in the safest manner.
        /// First serializes the data of the right hand into a string, then initializes the current parameters via >>
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

        /// \brief Goal heuristic function.(optional)
        ///
        /// distance = goalfn(config)
        ///
        /// Goal is complete when returns 0
        /// \param distance - distance to closest goal
        boost::function<dReal(const std::vector<dReal>&)> _goalfn;

        /// \brief optional, Distance metric between configuration spaces, two configurations are considered the same when this returns 0: distmetric(config1,config2)
        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

        typedef boost::function<bool(const std::vector<dReal>&, const std::vector<dReal>&, IntervalType, PlannerBase::ConfigurationListPtr)> CheckPathConstraintFn;
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
            \param configurations Optional argument that will hold the path between the two configurations if requested
        */
        CheckPathConstraintFn _checkpathconstraintsfn;

        /// \brief Samples a random configuration (mandatory)
        ///
        /// The dimension of the returned sample is the dimension of the configuration space.
        /// success = samplefn(newsample)
        boost::function<bool(std::vector<dReal>&)> _samplefn;

        /// \brief Samples a valid goal configuration (optional).
        ///
        /// If valid, the function should be called
        /// at every iteration. Any type of sampling probabilities and conditions can be encoded inside the function.
        /// The dimension of the returned sample is the dimension of the configuration space.
        /// success = samplegoalfn(newsample)
        boost::function<bool(std::vector<dReal>&)> _samplegoalfn;

        /// \brief Samples a valid initial configuration (optional).
        ///
        /// If valid, the function should be called
        /// at every iteration. Any type of sampling probabilities and conditions can be encoded inside the function.
        /// The dimension of the returned sample is the dimension of the configuration space.
        /// success = sampleinitialfn(newsample)
        boost::function<bool(std::vector<dReal>&)> _sampleinitialfn;

        /** \brief Returns a random configuration around a neighborhood (optional).
         
            _sampleneighfn(newsample,pCurSample,fRadius)

            \param pCurSample - the neighborhood to sample around
            \param  fRadius - specifies the max distance of sampling. The higher the value, the farther the samples will go
                              The distance metric can be arbitrary, but is usually PlannerParameters::pdistmetric.
            \return if sample was successfully generated return true, otherwise false
        */
        boost::function<bool(std::vector<dReal>&, const std::vector<dReal>&, dReal)> _sampleneighfn;

        /// \brief Sets the state of the robot. Default is active robot joints (mandatory).
        boost::function<void(const std::vector<dReal>&)> _setstatefn;
        /// \brief Gets the state of the robot. Default is active robot joints (mandatory).
        boost::function<void(std::vector<dReal>&)> _getstatefn;

        /** \brief  Computes the difference of two states.
           
            _diffstatefn(q1,q2) -> q1 -= q2
        
            An explicit difference function is necessary for correct interpolation when there are circular joints.
            Default is regular subtraction.
        */
        boost::function<void(std::vector<dReal>&,const std::vector<dReal>&)> _diffstatefn;

        /** \brief Adds a delta state to a curent state, acting like a next-nearest-neighbor function along a given direction.
            
            success = _neighstatefn(q,qdelta,fromgoal) -> q = Filter(q+qdelta)
            \param q the current state
            \param qdelta the delta to add
            \param fromgoal 1 if q is coming from a goal state, 0 if it is coming from an initial state
            
            In RRTs this is used for the extension operation. The new state is stored in the first parameter q.
            Note that the function can also add a filter to the final destination (like projecting onto a constraint manifold).
        */
        boost::function<bool(std::vector<dReal>&,const std::vector<dReal>&, int)> _neighstatefn;

        /// to specify multiple initial or goal configurations, put them into the vector in series
        /// (note: not all planners support multiple goals)
        std::vector<dReal> vinitialconfig, vgoalconfig;

        /// \brief the absolute limits of the configuration space.
        std::vector<dReal> _vConfigLowerLimit,_vConfigUpperLimit;

        /// \brief the discretization resolution of each dimension of the configuration space
        std::vector<dReal> _vConfigResolution;
        
        /// \brief a minimum distance between neighbors when searching. If 0 or less, planner chooses best step length
        dReal _fStepLength;

        /// \brief maximum number of iterations before the planner gives up. If 0 or less, planner chooses best iterations.
        int _nMaxIterations;

        /// \brief Specifies the planner that will perform the post-processing path smoothing before returning.
        ///
        /// If empty, will not path smooth the returned trajectories (used to measure algorithm time)
        std::string _sPathOptimizationPlanner;
        
        /// \brief The serialized planner parameters to pass to the path optimizer.
        ///
        /// For example: std::stringstream(_sPathOptimizationParameters) >> _parameters;
        std::string _sPathOptimizationParameters;

        /// \brief Extra parameters data that does not fit within this planner parameters structure, but is still important not to lose all the information.
        std::string _sExtraParameters;

        /// \brief Return the degrees of freedom of the planning configuration space
        virtual int GetDOF() const { return (int)_vConfigLowerLimit.size(); }

    protected:
        inline boost::shared_ptr<PlannerBase::PlannerParameters> shared_parameters() { return boost::static_pointer_cast<PlannerBase::PlannerParameters>(shared_from_this()); }
        inline boost::shared_ptr<PlannerBase::PlannerParameters const > shared_parameters_const() const { return boost::static_pointer_cast<PlannerBase::PlannerParameters const>(shared_from_this()); }

        /// output the planner parameters in a string (in XML format)
        /// don't use PlannerParameters as a tag!
        virtual bool serialize(std::ostream& O) const;
                               
        //@{ XML parsing functions, parses the default parameters
        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
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
        friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const PlannerParameters& v);
        /// expects \verbatim <PlannerParameters> \endverbatim to be the first token. Parses stream until \verbatim </PlannerParameters> \endverbatim reached
        friend OPENRAVE_API std::istream& operator>>(std::istream& I, PlannerParameters& v);
    };
    typedef boost::shared_ptr<PlannerBase::PlannerParameters> PlannerParametersPtr;
    typedef boost::shared_ptr<PlannerBase::PlannerParameters const> PlannerParametersConstPtr;
    typedef boost::weak_ptr<PlannerBase::PlannerParameters> PlannerParametersWeakPtr;

    PlannerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Planner, penv) {}
    virtual ~PlannerBase() {}

    /// \return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_Planner; }
    
    /// \brief Setup scene, robot, and properties of the plan, and reset all internal structures.
    /// \param probot The robot will be planning for.
    /// \param pparams The parameters of the planner, any class derived from PlannerParameters can be passed. The planner should copy these parameters for future instead of storing the pointer.
    virtual bool InitPlan(RobotBasePtr probot, PlannerParametersConstPtr pparams) = 0;

    /** \brief Setup scene, robot, and properties of the plan, and reset all structures with pparams.
    
        \param robot The robot will be planning for. Although the configuration space of the planner and the robot can be independent,
        the planner uses the robot to check for environment and self-collisions.
        In order to speed up computations further, planners can use the CO_ActiveDOFs collision checker option, which only focuses
        collision on the currently moving links in the robot.
        Even if the configuration space of the planner is different from the robot, the robot active DOFs must be set correctly (or at least have all dofs active)!

        \param isParameters The serialized form of the parameters. By default, this exists to allow third parties to
        pass information to planners without excplicitly knowning the format/internal structures used
        \return true if plan is initialized successfully and initial conditions are satisfied.
    */
    virtual bool InitPlan(RobotBasePtr robot, std::istream& isParameters);

    /** \brief Executes the main planner trying to solve for the goal condition.
    
        Fill ptraj with the trajectory of the planned path that the robot needs to execute
        \param ptraj The output trajectory the robot has to follow in order to successfully complete the plan. If this planner is a path optimizer, the trajectory can be used as an input for generating a smoother path. The trajectory is for the configuration degrees of freedom defined by the planner parameters.
        \param pOutStream If specified, planner will output any other special data
        \return true if planner is successful
    */
    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream = boost::shared_ptr<std::ostream>()) = 0;

    /// \brief return the internal parameters of the planner
    virtual PlannerParametersConstPtr GetParameters() const = 0;

protected:
    /** \brief Calls a planner to optimizes the trajectory path.

        The PlannerParameters structure passed into the optimization planner is
        constructed with the same freespace constraints as this planner.
        This function should always be called in PlanPath to post-process the trajectory.
        \param probot the robot this trajectory is meant for, also uses the robot for checking collisions.
        \param ptraj Initial trajectory to be smoothed is inputted. If optimization path succeeds, final trajectory output is set in this variable. The trajectory is for the configuration degrees of freedom defined by the planner parameters.
    */
    virtual bool _OptimizePath(RobotBasePtr probot, TrajectoryBasePtr ptraj);
    
private:
    virtual const char* GetHash() const { return OPENRAVE_PLANNER_HASH; }
};

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v);
OPENRAVE_API std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& v);

} // end namespace OpenRAVE

#endif
