#include "jolt_physics_shape_3d.hpp"

#include "conversion.hpp"

JoltPhysicsShape3D::~JoltPhysicsShape3D() = default;

Variant JoltPhysicsSphereShape3D::get_data() const {
	return radius;
}

void JoltPhysicsSphereShape3D::set_data(const Variant& p_data) {
	radius = p_data;
	jref = new JPH::SphereShape(radius);
}

Variant JoltPhysicsBoxShape3D::get_data() const {
	return half_extents;
}

void JoltPhysicsBoxShape3D::set_data(const Variant& p_data) {
	half_extents = p_data;
	jref = new JPH::BoxShape(to_jolt(half_extents));
}
