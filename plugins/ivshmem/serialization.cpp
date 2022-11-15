#include <type_traits>
#include <fcl/shape/geometric_shapes.h>

#include "serialization.hpp"
#include <openrave/openrave.h>

static_assert(std::is_same<fcl::FCL_REAL, double>::value, "FCL_REAL is not 64-bit float!");
static_assert(sizeof(void*) == 8, "Size of pointer is not 8 bytes!");

namespace ivshmem {

/// Serialization ====================================================================================================

template <>
uint64_t serialize<bool, false>(uint8_t* const mem, const bool& v) noexcept {
    uint8_t bval = v ? 1 : 0;
    ::memcpy(mem, &bval, sizeof(uint8_t));
    return sizeof(uint8_t);
}

uint64_t serialize(uint8_t* const mem, const char* str, size_t len) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, len);
    ::memcpy(mem + offset, str, len);
    offset += len;
    return offset;
}
uint64_t serialize(uint8_t* const mem, const std::string& str) noexcept {
    return serialize(mem, str.c_str(), str.size());
}
uint64_t serialize(uint8_t* const mem, const fcl::Vec3f& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v[0]);
    offset += serialize(mem + offset, v[1]);
    offset += serialize(mem + offset, v[2]);
    return offset;
}
size_t serialize(uint8_t* const mem, const fcl::Quaternion3f& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v[0]);
    offset += serialize(mem + offset, v[1]);
    offset += serialize(mem + offset, v[2]);
    offset += serialize(mem + offset, v[3]);
    return offset;
}
uint64_t serialize(uint8_t* const mem, const fcl::Triangle& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v[0]);
    offset += serialize(mem + offset, v[1]);
    offset += serialize(mem + offset, v[2]);
    return offset;
}
uint64_t serialize(uint8_t* const mem, const fcl::Transform3f& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v.getTranslation());
    offset += serialize(mem + offset, v.getQuatRotation());
    return offset;
}
uint64_t serialize(uint8_t* const mem, const fcl::AABB& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v.min_);
    offset += serialize(mem + offset, v.max_);
    return offset;
}
uint64_t serialize(uint8_t* const mem, const fcl::BVHModel<fcl::OBB>& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v.num_vertices);
    for (int i = 0; i < v.num_vertices; ++i) {
        offset += serialize(mem + offset, v.vertices[i]);
    }
    offset += serialize(mem + offset, v.num_tris);
    for (int i = 0; i < v.num_tris; ++i) {
        offset += serialize(mem + offset, v.tri_indices[i]);
    }
    return offset;
}
uint64_t serialize(uint8_t* const mem, const fcl::Box& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v.side);
    return offset;
}
uint64_t serialize(uint8_t* const mem, const fcl::Cylinder& v) noexcept {
    uint64_t offset = 0;
    offset += serialize(mem + offset, v.radius);
    offset += serialize(mem + offset, v.lz);
    return offset;
}

uint64_t serialize(uint8_t* const mem, const std::shared_ptr<const fcl::CollisionGeometry>& v) noexcept {
    uint64_t offset = 0;
    offset += serialize<uint16_t>(mem, v->getNodeType());
    RAVELOG_INFO("Node type is %d", static_cast<uint16_t>(v->getNodeType()));
    switch (v->getNodeType()) {
    case fcl::NODE_TYPE::BV_OBB: {
        offset += serialize(mem, *static_cast<const fcl::BVHModel<fcl::OBB>*>(v.get()));
        break;
    }
    case fcl::NODE_TYPE::GEOM_BOX: {
        offset += serialize(mem, *static_cast<const fcl::Box*>(v.get()));
        break;
    }
    case fcl::NODE_TYPE::GEOM_CYLINDER: {
        offset += serialize(mem, *static_cast<const fcl::Cylinder*>(v.get()));
        break;
    }
    default: {
        RAVELOG_WARN("Unsupported object type %d or node type %d.", v->getObjectType(), v->getNodeType());
        break;
    }
    }
    return offset;
}

uint64_t serialize(uint8_t* const mem, const fcl::CollisionObject& obj) noexcept {
    size_t offset = 0;
    offset += serialize(mem + offset, obj.getTransform());
    offset += serialize(mem + offset, obj.collisionGeometry());
    return offset;
}

uint8_t Cast(fcl::GJKSolverType v) {
    switch (v) {
    case fcl::GJKSolverType::GST_LIBCCD: return 0;
    case fcl::GJKSolverType::GST_INDEP: return 1;
    }
    return -1;
}

uint64_t serialize(uint8_t* const mem, const fcl::CollisionRequest& v) noexcept {
    size_t offset = 0;
    offset += serialize(mem + offset, v.num_max_contacts);
    offset += serialize(mem + offset, v.enable_contact);
    //offset += serialize(mem + offset, v.num_max_cost_sources);
    //offset += serialize(mem + offset, v.enable_cost);
    //offset += serialize(mem + offset, v.use_approximate_cost);
    //offset += serialize(mem + offset, Cast(v.gjk_solver_type));
    return offset;
}

uint64_t serialize(uint8_t* const mem, const fcl::DistanceRequest& v) noexcept {
    size_t offset = 0;
    offset += serialize(mem + offset, v.enable_nearest_points);
    offset += serialize(mem + offset, v.rel_err);
    offset += serialize(mem + offset, v.abs_err);
    //offset += serialize(mem + offset, Cast(v.gjk_solver_type));
    return offset;
}

/// Deserialization ==================================================================================================

template <>
uint64_t deserialize<bool>(const uint8_t* const mem, bool& v) {
    uint8_t val;
    uint64_t offset = deserialize<uint8_t>(mem, val);
    v = (val != 0);
    return offset;
}

uint64_t deserialize(const uint8_t* const mem, fcl::Vec3f& v) {
    uint64_t offset = 0;
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v[0]);
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v[1]);
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v[2]);
    return offset;
}
uint64_t deserialize(const uint8_t* const mem, fcl::Quaternion3f& v) {
    uint64_t offset = 0;
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v.getW());
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v.getX());
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v.getY());
    offset += deserialize<fcl::FCL_REAL>(mem + offset, v.getZ());
    return offset;
}
uint64_t deserialize(const uint8_t* const mem, fcl::Contact& v) {
    uint64_t offset = 0;
    offset = deserialize<decltype(v.b1)>(mem + offset, v.b1);
    offset = deserialize<decltype(v.b2)>(mem + offset, v.b2);
    offset = deserialize<decltype(v.normal)>(mem + offset, v.normal);
    offset = deserialize<decltype(v.pos)>(mem + offset, v.pos);
    offset = deserialize<decltype(v.penetration_depth)>(mem + offset, v.penetration_depth);
    return offset;
}
uint64_t deserialize(const uint8_t* const mem, fcl::CostSource& v) {
    uint64_t offset = 0;
    offset = deserialize<decltype(v.aabb_min)>(mem + offset, v.aabb_min);
    offset = deserialize<decltype(v.aabb_max)>(mem + offset, v.aabb_max);
    offset = deserialize<decltype(v.cost_density)>(mem + offset, v.cost_density);
    offset = deserialize<decltype(v.total_cost)>(mem + offset, v.total_cost);
    return offset;
}
uint64_t deserialize(const uint8_t* const mem, fcl::CollisionResult& v) {
    uint64_t offset = 0;
    v.clear();
    size_t numContacts = 0;
    offset += deserialize<size_t>(mem + offset, numContacts);
    while (numContacts-- > 0) {
        fcl::Contact contact;
        offset += deserialize<fcl::Contact>(mem + offset, contact);
        v.addContact(std::move(contact));
    }
    size_t numCostSource = 0;
    offset += deserialize<size_t>(mem + offset, numCostSource);
    while (numCostSource-- > 0) {
        fcl::CostSource costSource;
        offset += deserialize<fcl::CostSource>(mem + offset, costSource);
        v.addCostSource(std::move(costSource), std::numeric_limits<size_t>::max());
    }
    return offset;
}
uint64_t deserialize(const uint8_t* const mem, fcl::DistanceResult& v) {
    uint64_t offset = 0;
    v.clear();
    offset += deserialize<decltype(v.min_distance)>(mem + offset, v.min_distance);
    offset = deserialize<decltype(v.b1)>(mem + offset, v.b1);
    offset = deserialize<decltype(v.b2)>(mem + offset, v.b2);
    return offset;
}

} // namespace ivshmem
