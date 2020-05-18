#ifndef PLUGINS_POSTUREDESCRIBER_POSTURESUPPORTTYPES_H
#define PLUGINS_POSTUREDESCRIBER_POSTURESUPPORTTYPES_H
#include <cstdint> // uint16_t
#include <type_traits> // underlying_type

namespace OpenRAVE {

// https://stackoverflow.com/questions/12059774/c11-standard-conformant-bitmasks-using-enum-class
enum class NeighbouringTwoJointsRelation : uint16_t {
    NTJR_Unknown                 = 0x0,
    NTJR_Parallel                = 0x1,
    NTJR_Perpendicular           = 0x2,
    NTJR_Intersect               = 0x4,
    NTJR_Overlap                 = NTJR_Intersect | NTJR_Parallel,      // 0x5
    NTJR_Intersect_Perpendicular = NTJR_Intersect | NTJR_Perpendicular, // 0x6
};

enum class RobotPostureSupportType : uint16_t {
    RPST_NoSupport  = 0x0,
    RPST_6R_General = 0x1,
    RPST_4R_Type_A  = 0x2,
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

#endif // PLUGINS_POSTUREDESCRIBER_SUPPORTTYPES_H
