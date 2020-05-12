#ifndef PLUGINS_FKCOMPUTERS_NEIGHBOURINGTWOJOINTSRELATIONS
#define PLUGINS_FKCOMPUTERS_NEIGHBOURINGTWOJOINTSRELATIONS
#include <cstdint> // uint16_t

namespace OpenRAVE {

// https://stackoverflow.com/questions/12059774/c11-standard-conformant-bitmasks-using-enum-class
enum class NeighbouringTwoJointsRelations : uint16_t {
    NTJR_UNKNOWN                 = 0x0,
    NTJR_PARALLEL                = 0x1,
    NTJR_PERPENDICULAR           = 0x2,
    NTJR_INTERSECT               = 0x4,
    NTJR_OVERLAP                 = NTJR_INTERSECT | NTJR_PARALLEL,      // 0x5
    NTJR_INTERSECT_PERPENDICULAR = NTJR_INTERSECT | NTJR_PERPENDICULAR, // 0x6
};

inline constexpr NeighbouringTwoJointsRelations operator&(NeighbouringTwoJointsRelations x, NeighbouringTwoJointsRelations y) {
return static_cast<NeighbouringTwoJointsRelations>(static_cast<int>(x) & static_cast<int>(y));
}

inline constexpr NeighbouringTwoJointsRelations operator|(NeighbouringTwoJointsRelations x, NeighbouringTwoJointsRelations y) {
return static_cast<NeighbouringTwoJointsRelations>(static_cast<int>(x) | static_cast<int>(y));
}

inline NeighbouringTwoJointsRelations& operator&=(NeighbouringTwoJointsRelations& x, NeighbouringTwoJointsRelations y) {
    return x = x & y;
}

inline NeighbouringTwoJointsRelations& operator|=(NeighbouringTwoJointsRelations& x, NeighbouringTwoJointsRelations y) {
    return x = x | y;
}

} // namespace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_NEIGHBOURINGTWOJOINTSRELATIONS
