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

Variant JoltPhysicsConvexPolygonShape3D::get_data() const {
	return vertices;
}

void JoltPhysicsConvexPolygonShape3D::set_data(const Variant& p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR3_ARRAY);
	vertices = p_data;
	vertices_changed();
}

void JoltPhysicsConvexPolygonShape3D::vertices_changed() {
	const int64_t num_vertices = vertices.size();
	ERR_FAIL_COND(num_vertices == 0);

	JPH::Array<JPH::Vec3> jolt_vertices;
	jolt_vertices.reserve((size_t)num_vertices);

	const Vector3* vertices_begin = &vertices[0];
	const Vector3* vertices_end = vertices_begin + num_vertices;

	for (const Vector3* vertex = vertices_begin; vertex != vertices_end; ++vertex) {
		jolt_vertices.emplace_back(vertex->x, vertex->y, vertex->z);
	}

	JPH::ConvexHullShapeSettings shape_settings(jolt_vertices);
	JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
	ERR_FAIL_COND(shape_result.IsEmpty());

	jref = shape_result.Get();
}
