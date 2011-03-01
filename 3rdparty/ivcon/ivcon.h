#ifndef OPENRAVE_IVCON_H
#define OPENRAVE_IVCON_H

#include <vector>

namespace ivcon {

bool ReadFile(const char* pfilename, std::vector<float>& vertices, std::vector<int>& indices);

}

#endif
