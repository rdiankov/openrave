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

// Writes an arithmetic value in its binary form to the output stream.
template <typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
size_t Serialize(std::ostream& os, T value) {
    os.write(reinterpret_cast<char*>(&value), sizeof(T));
    return sizeof(T);
}

template <>
size_t Serialize<bool>(std::ostream&, bool);

// Writes a length-prefixed, null-terminated string to the output stream.
// Always writes a null terminator even if string length is 0.
size_t Serialize(std::ostream& , const char*, size_t length);
size_t Serialize(std::ostream& , const std::string&);

size_t Serialize(std::ostream&, const fcl::Vec3f&);
size_t Serialize(std::ostream&, const fcl::Quaternion3f&);
size_t Serialize(std::ostream&, const fcl::Triangle&);
size_t Serialize(std::ostream&, const fcl::Transform3f&);
size_t Serialize(std::ostream&, const fcl::AABB&);
size_t Serialize(std::ostream&, const fcl::BVHModel<fcl::OBB>&);

// Convenience overloads for collision geometry
size_t Serialize(std::ostream&, const std::shared_ptr<const fcl::CollisionGeometry>&);
size_t Serialize(std::ostream&, const fcl::CollisionGeometry*);
size_t Serialize(std::ostream&, const fcl::CollisionObject&);
size_t Serialize(std::ostream&, const fcl::CollisionRequest&);
size_t Serialize(std::ostream&, const fcl::DistanceRequest&);

/// Deserialization ==================================================================================================

/// A long, long time ago, FILE* pointers and C++ streams lived in harmony
/// But everything changed when the fire nation attacked, so we have to write these adapters now.
/// To use this filebuf, construct an istream with it: istream is(&fb);
/// The filebuf object must stay in scope for the duration of the deserialization.
std::filebuf OpenMemoryAsFile(void* memptr, size_t size);

template <typename T, typename U = typename std::remove_reference<T>::type>
inline U DeSerialize(std::istream& is) {
    U value;
    is.read(reinterpret_cast<char*>(&value), sizeof(U));
    return value;
}

template <>
bool DeSerialize<bool>(std::istream&);

template <>
fcl::Vec3f DeSerialize<fcl::Vec3f>(std::istream&);

template <>
fcl::Quaternion3f DeSerialize<fcl::Quaternion3f>(std::istream&);

template <>
fcl::Contact DeSerialize<fcl::Contact>(std::istream&);

template <>
fcl::CostSource DeSerialize<fcl::CostSource>(std::istream&);

template <>
fcl::CollisionResult DeSerialize<fcl::CollisionResult>(std::istream&);

template <>
fcl::DistanceResult DeSerialize<fcl::DistanceResult>(std::istream&);

} // namespace ivshmem

#endif // OPENRAVE_IVSHMEM_SERIALIZATION_HPP
