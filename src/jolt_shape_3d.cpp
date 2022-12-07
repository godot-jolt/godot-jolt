#include "jolt_shape_3d.hpp"

#include "conversion.hpp"
#include "utility_functions.hpp"

JoltShape3D::~JoltShape3D() = default;

void JoltShape3D::clear_data() {
	jref = nullptr;
}

Variant JoltSphereShape3D::get_data() const {
	return radius;
}

void JoltSphereShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::FLOAT);

	const float new_radius = p_data;

	const JPH::SphereShapeSettings shape_settings(new_radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create sphere shape with radius '{}'. "
			"Jolt returned the following error: '{}'.",
			new_radius,
			shape_result.GetError()
		)
	);

	jref = shape_result.Get();
	radius = new_radius;
}

void JoltSphereShape3D::clear_data() {
	JoltShape3D::clear_data();
	radius = 0.0f;
}

Variant JoltBoxShape3D::get_data() const {
	return half_extents;
}

void JoltBoxShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR3);

	const Vector3 new_half_extents = p_data;

	const JPH::BoxShapeSettings shape_settings(to_jolt(new_half_extents));
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create box shape with half extents '({}, {}, {})'. "
			"Jolt returned the following error: '{}'.",
			new_half_extents.x,
			new_half_extents.y,
			new_half_extents.z,
			shape_result.GetError()
		)
	);

	jref = shape_result.Get();
	half_extents = new_half_extents;
}

void JoltBoxShape3D::clear_data() {
	JoltShape3D::clear_data();
	half_extents.zero();
}

Variant JoltCapsuleShape3D::get_data() const {
	Dictionary dict;
	dict["height"] = height;
	dict["radius"] = radius;
	return dict;
}

void JoltCapsuleShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary dict = p_data;

	const Variant maybe_height = dict.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = dict.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	const float new_height = maybe_height;
	const float new_radius = maybe_radius;

	const JPH::CapsuleShapeSettings shape_settings(new_height / 2.0f, new_radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create capsule shape with height '{}' and radius '{}'. "
			"Jolt returned the following error: '{}'.",
			new_height,
			new_radius,
			shape_result.GetError()
		)
	);

	jref = shape_result.Get();
	height = new_height;
	radius = new_radius;
}

void JoltCapsuleShape3D::clear_data() {
	JoltShape3D::clear_data();
	height = 0.0f;
	radius = 0.0f;
}

Variant JoltCylinderShape3D::get_data() const {
	Dictionary dict;
	dict["height"] = height;
	dict["radius"] = radius;
	return dict;
}

void JoltCylinderShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary dict = p_data;

	const Variant maybe_height = dict.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = dict.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	const float new_height = maybe_height;
	const float new_radius = maybe_radius;

	const JPH::CylinderShapeSettings shape_settings(new_height / 2.0f, new_radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create cylinder shape with height '{}' and radius '{}'. "
			"Jolt returned the following error: '{}'.",
			new_height,
			new_radius,
			shape_result.GetError()
		)
	);

	jref = shape_result.Get();
	height = new_height;
	radius = new_radius;
}

void JoltCylinderShape3D::clear_data() {
	JoltShape3D::clear_data();
	height = 0.0f;
	radius = 0.0f;
}

Variant JoltConvexPolygonShape3D::get_data() const {
	return vertices;
}

void JoltConvexPolygonShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	PackedVector3Array new_vertices = p_data;

	const int64_t num_vertices = new_vertices.size();
	ERR_FAIL_COND(num_vertices == 0);

	JPH::Array<JPH::Vec3> jolt_vertices;
	jolt_vertices.reserve((size_t)num_vertices);

	const Vector3* vertices_begin = &new_vertices[0];
	const Vector3* vertices_end = vertices_begin + num_vertices;

	for (const Vector3* vertex = vertices_begin; vertex != vertices_end; ++vertex) {
		jolt_vertices.emplace_back(vertex->x, vertex->y, vertex->z);
	}

	const JPH::ConvexHullShapeSettings shape_settings(jolt_vertices);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create convex polygon shape with vertex count '{}'. "
			"Jolt returned the following error: '{}'.",
			new_vertices.size(),
			shape_result.GetError()
		)
	);

	jref = shape_result.Get();
	vertices = std::move(new_vertices);
}

void JoltConvexPolygonShape3D::clear_data() {
	JoltShape3D::clear_data();
	vertices.clear();
}
