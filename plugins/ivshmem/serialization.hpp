#ifndef OPENRAVE_IVSHMEM_SERIALIZATION_HPP
#define OPENRAVE_IVSHMEM_SERIALIZATION_HPP

#include <ext/stdio_filebuf.h>
#include <stdio.h>

#include <fstream>
#include <iosfwd>

#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>

namespace ivshmem {

/// Serialization ====================================================================================================

// Basic template for primitive types.
template <typename T, typename std::enable_if<std::is_arithmetic<T>::value, bool>::type = true>
uint64_t serialize(uint8_t* const mem, const T& v) noexcept {
    ::memcpy(mem, &v, sizeof(T));
    return sizeof(T);
}

template <>
uint64_t serialize<bool, false>(uint8_t* const, const bool&) noexcept;
uint64_t serialize(uint8_t* const, const char*, size_t) noexcept;
uint64_t serialize(uint8_t* const, const std::string&) noexcept;
uint64_t serialize(uint8_t* const, const fcl::Vec3f&) noexcept;
uint64_t serialize(uint8_t* const, const fcl::Quaternion3f&) noexcept;
uint64_t serialize(uint8_t* const, const fcl::Triangle&) noexcept;
uint64_t serialize(uint8_t* const, const fcl::Transform3f&) noexcept;
uint64_t serialize(uint8_t* const, const fcl::AABB&) noexcept;

// Convenience overloads for collision geometry
size_t serialize(uint8_t* const, const fcl::CollisionObject&) noexcept;
size_t serialize(uint8_t* const, const fcl::CollisionRequest&) noexcept;
size_t serialize(uint8_t* const, const fcl::DistanceRequest&) noexcept;

/// Deserialization ==================================================================================================

template <typename T, typename U = typename std::remove_reference<T>::type>
inline uint64_t deserialize(const uint8_t* const mem, U& value) {
    memcpy(&value, mem, sizeof(U));
    return sizeof(U);
}

template <>
uint64_t deserialize<bool>(const uint8_t* const, bool&);

uint64_t deserialize(const uint8_t* const, fcl::Vec3f&);
uint64_t deserialize(const uint8_t* const, fcl::Quaternion3f&);
uint64_t deserialize(const uint8_t* const, fcl::Contact&);
uint64_t deserialize(const uint8_t* const, fcl::CostSource&);
uint64_t deserialize(const uint8_t* const, fcl::CollisionResult&);
uint64_t deserialize(const uint8_t* const, fcl::DistanceResult&);

} // namespace ivshmem

#endif // OPENRAVE_IVSHMEM_SERIALIZATION_HPP
