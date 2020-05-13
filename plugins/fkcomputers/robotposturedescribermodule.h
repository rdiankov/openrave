#ifndef PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBERMODULE_H
#define PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBERMODULE_H

#include <openrave/fksolver.h>

namespace OpenRAVE {

class RobotPostureDescriberModule : public ModuleBase
{
public:
    RobotPostureDescriberModule() = delete; // disable default constructor
    RobotPostureDescriberModule(const EnvironmentBasePtr& penv);
    virtual ~RobotPostureDescriberModule() = default;

private:
    /// \brief Python `SendCommand` API that loads a robot posture describer onto a (base link, end-effector link) pair, or onto a manipulator that prescribes the pair
    bool _LoadRobotPostureDescriberCommand(std::ostream& ssout, std::istream& ssin);
};

} // namepspace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBERMODULE_H