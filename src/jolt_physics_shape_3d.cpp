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

Variant JoltPhysicsCapsuleShape3D::get_data() const {
	Dictionary dict;
	dict["height"] = height;
	dict["radius"] = radius;
	return dict;
}

void JoltPhysicsCapsuleShape3D::set_data(const Variant& p_data) {
	const Dictionary dict = p_data;

	const Variant maybe_height = dict.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = dict.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	height = maybe_height;
	radius = maybe_radius;

	jref = new JPH::CapsuleShape(height / 2.0f, radius);
}

Variant JoltPhysicsCylinderShape3D::get_data() const {
	Dictionary dict;
	dict["height"] = height;
	dict["radius"] = radius;
	return dict;
}

void JoltPhysicsCylinderShape3D::set_data(const Variant& p_data) {
	const Dictionary dict = p_data;

	const Variant maybe_height = dict.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = dict.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	height = maybe_height;
	radius = maybe_radius;

	jref = new JPH::CylinderShape(height / 2.0f, radius);
}
