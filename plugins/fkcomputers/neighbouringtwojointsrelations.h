#ifndef PLUGINS_FKCOMPUTERS_NEIGHBOURINGTWOJOINTSRELATIONS
#define PLUGINS_FKCOMPUTERS_NEIGHBOURINGTWOJOINTSRELATIONS
#include <cstdint> // uint16_t
#include <type_traits> // underlying_type

namespace OpenRAVE {

// https://stackoverflow.com/questions/12059774/c11-standard-conformant-bitmasks-using-enum-class
enum class NeighbouringTwoJointsRelation : uint16_t {
    NTJR_UNKNOWN                 = 0x0,
    NTJR_PARALLEL                = 0x1,
    NTJR_PERPENDICULAR           = 0x2,
    NTJR_INTERSECT               = 0x4,
    NTJR_OVERLAP                 = NTJR_INTERSECT | NTJR_PARALLEL,      // 0x5
    NTJR_INTERSECT_PERPENDICULAR = NTJR_INTERSECT | NTJR_PERPENDICULAR, // 0x6
};

enum class RobotPostureSupportType : uint16_t {
    RPST_NOSUPPORT   = 0x0,
    RPST_6R_GENERAL  = 0x1,
    RPST_4R_SPECIAL_0  = 0x2,
};

template <typename T>
inline constexpr T operator&(T x, T y)
{
    using UT = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<UT>(x) & static_cast<UT>(y));
}

template <typename T>
inline constexpr T operator|(T x, T y)
{
    using UT = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<UT>(x) | static_cast<UT>(y));
}

template <typename T>
inline T operator&=(T& x, T y)
{
    return x = x & y;
}

template <typename T>
inline T operator|=(T& x, T y)
{
    return x = x | y;
}

} // namespace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_NEIGHBOURINGTWOJOINTSRELATIONS
