#include "serialization.hpp"

namespace ivshmem {

/// Serialization ====================================================================================================

template <>
size_t Serialize<bool>(std::ostream& os, bool value) {
    return Serialize(os, uint8_t(value ? 1 : 0));
}

size_t Serialize(std::ostream& os, const char* str, size_t length) {
    size_t size = 0;
    size += Serialize(os, length);
    os.write(str, length);
    size += length;
    return size;
}

size_t Serialize(std::ostream& os, const std::string& str) {
    return Serialize(os, str.c_str(), str.size());
}

size_t Serialize(std::ostream& os, const fcl::Vec3f& v) {
    size_t size = 0;
    size += Serialize(os, v[0]);
    size += Serialize(os, v[1]);
    size += Serialize(os, v[2]);
    return size;
}

size_t Serialize(std::ostream& os, const fcl::Quaternion3f& v) {
    size_t size = 0;
    size += Serialize(os, v[0]);
    size += Serialize(os, v[1]);
    size += Serialize(os, v[2]);
    size += Serialize(os, v[3]);
    return size;
}

size_t Serialize(std::ostream& os, const fcl::Triangle& v) {
    size_t size = 0;
    size += Serialize(os, v[0]);
    size += Serialize(os, v[1]);
    size += Serialize(os, v[2]);
    return size;
}

size_t Serialize(std::ostream& os, const fcl::Transform3f& v) {
    size_t size = 0;
    size += Serialize(os, v.getTranslation());
    size += Serialize(os, v.getQuatRotation());
    return size;
}

size_t Serialize(std::ostream& os, const fcl::AABB& v) {
    size_t size = 0;
    size += Serialize(os, v.min_);
    size += Serialize(os, v.max_);
    return size;
}

size_t Serialize(std::ostream& os, const fcl::BVHModel<fcl::OBB>& v) {
    size_t size = 0;
    size += Serialize(os, v.num_vertices);
    for (decltype(v.num_vertices) i = 0; i < v.num_vertices; ++i) {
        size += Serialize(os, v.vertices[i]);
    }
    size += Serialize(os, v.num_tris);
    for (decltype(v.num_tris) i = 0; i < v.num_tris; ++i) {
        size += Serialize(os, v.tri_indices[i]);
    }
    return size;
}

size_t Serialize(std::ostream& os, const std::shared_ptr<const fcl::CollisionGeometry>& v) {
    const fcl::BVHModel<fcl::OBB>* const ptr = static_cast<const fcl::BVHModel<fcl::OBB>* const>(v.get());
    return Serialize(os, *ptr);
}

size_t Serialize(std::ostream& os, const fcl::CollisionGeometry* v) {
    const fcl::BVHModel<fcl::OBB>* obj = static_cast<const fcl::BVHModel<fcl::OBB>*>(v);
    return Serialize(os, *obj);
}

size_t Serialize(std::ostream& os, const fcl::CollisionObject& obj) {
    size_t size = 0;
    size += Serialize(os, obj.getTransform());
    size += Serialize(os, obj.collisionGeometry());
    return size;
}

uint8_t Cast(fcl::GJKSolverType v) {
    switch (v) {
    case fcl::GJKSolverType::GST_LIBCCD: return 0;
    case fcl::GJKSolverType::GST_INDEP: return 1;
    }
    return -1;
}

size_t Serialize(std::ostream& os, const fcl::CollisionRequest& v) {
    size_t size = 0;
    size += Serialize(os, v.num_max_contacts);
    size += Serialize(os, v.enable_contact);
    //size += Serialize(os, v.num_max_cost_sources);
    //size += Serialize(os, v.enable_cost);
    //size += Serialize(os, v.use_approximate_cost);
    //size += Serialize(os, Cast(v.gjk_solver_type));
    return size;
}

size_t Serialize(std::ostream& os, const fcl::DistanceRequest& v) {
    size_t size = 0;
    size += Serialize(os, v.enable_nearest_points);
    size += Serialize(os, v.rel_err);
    size += Serialize(os, v.abs_err);
    //size += Serialize(os, Cast(v.gjk_solver_type));
    return size;
}

/// Deserialization ==================================================================================================

std::filebuf OpenMemoryAsFile(void* memptr, size_t size) {
    FILE* fileptr = ::fmemopen(memptr, size, "r");
    return __gnu_cxx::stdio_filebuf<char>(fileptr, std::ios::binary, size);
}

template <>
bool DeSerialize<bool>(std::istream& is) {
    return (DeSerialize<uint8_t>(is) != 0);
}

template <>
fcl::Vec3f DeSerialize<fcl::Vec3f>(std::istream& is) {
    fcl::Vec3f v;
    v[0] = DeSerialize<decltype(v[0])>(is);
    v[1] = DeSerialize<decltype(v[1])>(is);
    v[2] = DeSerialize<decltype(v[2])>(is);
    return v;
}

template <>
fcl::Quaternion3f DeSerialize<fcl::Quaternion3f>(std::istream& is) {
    fcl::Quaternion3f v;
    v.getW() = DeSerialize<decltype(v.getW())>(is);
    v.getX() = DeSerialize<decltype(v.getX())>(is);
    v.getY() = DeSerialize<decltype(v.getY())>(is);
    v.getZ() = DeSerialize<decltype(v.getZ())>(is);
    return v;
}

template <>
fcl::Contact DeSerialize<fcl::Contact>(std::istream& is) {
    fcl::Contact v;
    v.b1 = DeSerialize<decltype(v.b1)>(is);
    v.b2 = DeSerialize<decltype(v.b2)>(is);
    v.normal = DeSerialize<decltype(v.normal)>(is);
    v.pos = DeSerialize<decltype(v.pos)>(is);
    v.penetration_depth = DeSerialize<decltype(v.penetration_depth)>(is);
    return v;
}

template <>
fcl::CostSource DeSerialize<fcl::CostSource>(std::istream& is) {
    fcl::CostSource v;
    v.aabb_min = DeSerialize<decltype(v.aabb_min)>(is);
    v.aabb_max = DeSerialize<decltype(v.aabb_max)>(is);
    v.cost_density = DeSerialize<decltype(v.cost_density)>(is);
    v.total_cost = DeSerialize<decltype(v.total_cost)>(is);
    return v;
}

template <>
fcl::CollisionResult DeSerialize<fcl::CollisionResult>(std::istream& is) {
    fcl::CollisionResult result;
    result.clear();
    size_t numContacts = DeSerialize<size_t>(is);
    while (numContacts-- > 0) {
        result.addContact(DeSerialize<fcl::Contact>(is));
    }
    size_t numCostSources = DeSerialize<size_t>(is);
    while (numCostSources-- > 0) {
        result.addCostSource(DeSerialize<fcl::CostSource>(is), std::numeric_limits<size_t>::max());
    }
    return result;
}

template <>
fcl::DistanceResult DeSerialize<fcl::DistanceResult>(std::istream& is) {
    fcl::DistanceResult result;
    result.clear();
    result.min_distance = DeSerialize<decltype(result.min_distance)>(is);
    result.b1 = DeSerialize<decltype(result.b1)>(is);
    result.b2 = DeSerialize<decltype(result.b2)>(is);
    return result;
}

} // namespace ivshmem
