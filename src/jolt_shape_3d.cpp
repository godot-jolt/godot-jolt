#include "jolt_shape_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "variant.hpp"

JoltShape3D::~JoltShape3D() = default;

JPH::ShapeRefC JoltShape3D::with_scale(
	[[maybe_unused]] const JPH::ShapeRefC& p_shape,
	[[maybe_unused]] const Vector3& p_scale
) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::ScaledShapeSettings shape_settings(p_shape, to_jolt(p_scale));
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to scale shape with scale '{}'. "
			"Jolt returned the following error: '{}'.",
			p_scale,
			shape_result.GetError()
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShape3D::with_basis_origin(
	const JPH::ShapeRefC& p_shape,
	const Basis& p_basis,
	const Vector3& p_origin
) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::RotatedTranslatedShapeSettings shape_settings(
		to_jolt(p_origin),
		to_jolt(p_basis),
		p_shape
	);

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to offset shape with basis '{}' and origin '{}'. "
			"Jolt returned the following error: '{}'.",
			p_basis,
			p_origin,
			shape_result.GetError()
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShape3D::with_transform(
	const JPH::ShapeRefC& p_shape,
	const Transform3D& p_transform
) {
	JPH::ShapeRefC shape = p_shape;

	const Vector3 scale_squared(
		p_transform.basis[Vector3::AXIS_X].length_squared(),
		p_transform.basis[Vector3::AXIS_Y].length_squared(),
		p_transform.basis[Vector3::AXIS_Z].length_squared()
	);

	if (scale_squared != Vector3(1.0f, 1.0f, 1.0f)) {
		const Vector3 scale(
			Math::sqrt(scale_squared.x),
			Math::sqrt(scale_squared.y),
			Math::sqrt(scale_squared.z)
		);

		shape = with_scale(shape, scale);
	}

	if (p_transform != Transform3D()) {
		const Transform3D transform_normalized = p_transform.orthonormalized();
		shape = with_basis_origin(shape, transform_normalized.basis, transform_normalized.origin);
	}

	return shape;
}

JPH::ShapeRefC JoltShape3D::with_center_of_mass_offset(
	const JPH::ShapeRefC& p_shape,
	const Vector3& p_offset
) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::OffsetCenterOfMassShapeSettings shape_settings(to_jolt(p_offset), p_shape);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to offset center of mass with offset '{}'. "
			"Jolt returned the following error: '{}'.",
			p_offset,
			shape_result.GetError()
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShape3D::with_center_of_mass(
	const JPH::ShapeRefC& p_shape,
	const Vector3& p_center_of_mass
) {
	ERR_FAIL_NULL_D(p_shape);

	const Vector3 center_of_mass_inner = to_godot(p_shape->GetCenterOfMass());
	const Vector3 center_of_mass_offset = p_center_of_mass - center_of_mass_inner;

	if (center_of_mass_offset == Vector3()) {
		return p_shape;
	}

	return with_center_of_mass_offset(p_shape, center_of_mass_offset);
}

void JoltShape3D::clear_data() {
	jolt_ref = nullptr;
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

	jolt_ref = shape_result.Get();
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
			"Failed to create box shape with half extents '{}'. "
			"Jolt returned the following error: '{}'.",
			new_half_extents,
			shape_result.GetError()
		)
	);

	jolt_ref = shape_result.Get();
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

	jolt_ref = shape_result.Get();
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

	jolt_ref = shape_result.Get();
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

	if (num_vertices == 0) {
		return;
	}

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

	jolt_ref = shape_result.Get();
	vertices = std::move(new_vertices);
}

void JoltConvexPolygonShape3D::clear_data() {
	JoltShape3D::clear_data();
	vertices.clear();
}
