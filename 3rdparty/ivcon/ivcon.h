#ifndef OPENRAVE_IVCON
#define OPENRAVE_IVCON

#include <vector>

namespace ivcon {

bool ReadFile(const char* pfilename, , std::vector<float>& vertices, std::vector<int>& indices);

}

#endif
