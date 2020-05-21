#ifndef PLUGINS_FKCOMPUTERS_POSTUREDESCRIBERMODULE_H
#define PLUGINS_FKCOMPUTERS_POSTUREDESCRIBERMODULE_H

#include "posturedescriberinterface.h"

namespace OpenRAVE {

class PostureDescriberModule : public ModuleBase
{
public:
    PostureDescriberModule() = delete; // disable default constructor
    PostureDescriberModule(const EnvironmentBasePtr& penv);
    virtual ~PostureDescriberModule() = default;

private:
    /// \brief Python `SendCommand` API that loads a robot posture describer onto a (base link, end-effector link) pair, or onto a manipulator that prescribes the pair
    bool _LoadPostureDescriberCommand(std::ostream& ssout, std::istream& ssin);
};

} // namepspace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_POSTUREDESCRIBERMODULE_H