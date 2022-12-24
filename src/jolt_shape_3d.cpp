#include "jolt_shape_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "utility_functions.hpp"
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
	Transform3D transform = p_transform;

	if (transform == Transform3D()) {
		return shape;
	}

	const Vector3 scale_squared(
		transform.basis[Vector3::AXIS_X].length_squared(),
		transform.basis[Vector3::AXIS_Y].length_squared(),
		transform.basis[Vector3::AXIS_Z].length_squared()
	);

	if (scale_squared != Vector3(1.0f, 1.0f, 1.0f)) {
		const Vector3 scale(
			Math::sqrt(scale_squared.x),
			Math::sqrt(scale_squared.y),
			Math::sqrt(scale_squared.z)
		);

		shape = with_scale(shape, scale);
		transform = transform.orthonormalized();
	}

	return with_basis_origin(shape, transform.basis, transform.origin);
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
	jolt_ref = nullptr;
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
	jolt_ref = nullptr;
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

	const float half_height = new_height / 2.0f;
	const float clamped_height = max(half_height - new_radius, CMP_EPSILON);

	const JPH::CapsuleShapeSettings shape_settings(clamped_height, new_radius);
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
	jolt_ref = nullptr;
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
	jolt_ref = nullptr;
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
	const int64_t vertex_count = new_vertices.size();

	if (vertex_count == 0) {
		return;
	}

	JPH::Array<JPH::Vec3> jolt_vertices;
	jolt_vertices.reserve((size_t)vertex_count);

	const Vector3* vertices_begin = &new_vertices[0];
	const Vector3* vertices_end = vertices_begin + vertex_count;

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
			vertex_count,
			shape_result.GetError()
		)
	);

	jolt_ref = shape_result.Get();
	vertices = std::move(new_vertices);
}

void JoltConvexPolygonShape3D::clear_data() {
	jolt_ref = nullptr;
	vertices.clear();
}

Variant JoltConcavePolygonShape3D::get_data() const {
	Dictionary dict;
	dict["faces"] = faces;
	dict["backface_collision"] = backface_collision;
	return dict;
}

void JoltConcavePolygonShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary dict = p_data;

	const Variant maybe_faces = dict.get("faces", {});
	ERR_FAIL_COND(maybe_faces.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	const Variant maybe_backface_collision = dict.get("backface_collision", {});
	ERR_FAIL_COND(maybe_backface_collision.get_type() != Variant::BOOL);

	PackedVector3Array new_faces = maybe_faces;
	const bool new_backface_collision = maybe_backface_collision;

	const auto vertex_count = (size_t)new_faces.size();
	const size_t excess_vertex_count = vertex_count % 3;

	ERR_FAIL_COND_MSG(
		excess_vertex_count != 0,
		"Failed to create concave polygon shape with vertex count '{}'. "
		"Expected a vertex count divisible by 3."
	);

	const size_t face_count = vertex_count / 3;

	if (face_count == 0) {
		return;
	}

	JPH::TriangleList jolt_faces;
	jolt_faces.reserve(face_count);

	const Vector3* faces_begin = &new_faces[0];
	const Vector3* faces_end = faces_begin + vertex_count;

	for (const Vector3* vertex = faces_begin; vertex != faces_end; vertex += 3) {
		const Vector3* v0 = vertex + 0;
		const Vector3* v1 = vertex + 1;
		const Vector3* v2 = vertex + 2;

		jolt_faces.emplace_back(
			JPH::Float3(v2->x, v2->y, v2->z),
			JPH::Float3(v1->x, v1->y, v1->z),
			JPH::Float3(v0->x, v0->y, v0->z)
		);

		if (new_backface_collision) {
			jolt_faces.emplace_back(
				JPH::Float3(v0->x, v0->y, v0->z),
				JPH::Float3(v1->x, v1->y, v1->z),
				JPH::Float3(v2->x, v2->y, v2->z)
			);
		}
	}

	const JPH::MeshShapeSettings shape_settings(jolt_faces);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create concave polygon shape with vertex count '{}'. "
			"Jolt returned the following error: '{}'.",
			vertex_count,
			shape_result.GetError()
		)
	);

	jolt_ref = shape_result.Get();
	faces = std::move(new_faces);
	backface_collision = new_backface_collision;
}

void JoltConcavePolygonShape3D::clear_data() {
	jolt_ref = nullptr;
	faces.clear();
	backface_collision = false;
}

Variant JoltHeightMapShape3D::get_data() const {
	Dictionary dict;
	dict["width"] = width;
	dict["depth"] = depth;
	dict["heights"] = heights;
	return dict;
}

void JoltHeightMapShape3D::set_data(const Variant& p_data) {
	clear_data();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary dict = p_data;

	const Variant maybe_heights = dict.get("heights", {});
	ERR_FAIL_COND(maybe_heights.get_type() != Variant::PACKED_FLOAT32_ARRAY);

	const Variant maybe_width = dict.get("width", {});
	ERR_FAIL_COND(maybe_width.get_type() != Variant::INT);

	const Variant maybe_depth = dict.get("depth", {});
	ERR_FAIL_COND(maybe_depth.get_type() != Variant::INT);

	const PackedFloat32Array new_heights = maybe_heights;
	const int new_width = maybe_width;
	const int new_depth = maybe_depth;

	const auto height_count = (int)new_heights.size();

	if (height_count == 0) {
		return;
	}

	// HACK(mihe): A height map shape will have a width or depth of 2 while it's transitioning from
	// its default state. Since Jolt doesn't support non-square height maps, and it's unlikely that
	// anyone would actually want a height map of such small dimensions, we silently ignore any such
	// request in order to not display an error every single time we create a shape of this type.
	if (new_width == 2 || new_depth == 2) {
		return;
	}

	ERR_FAIL_COND_MSG(
		height_count != new_width * new_depth,
		vformat(
			"Failed to create height map shape with width '{}', depth '{}' and height count '{}'. "
			"Expected height count to equal width multiplied by depth.",
			new_width,
			new_depth,
			height_count
		)
	);

	ERR_FAIL_COND_MSG(
		new_width != new_depth,
		vformat(
			"Failed to create height map shape with width '{}', depth '{}' and height count '{}'. "
			"Height maps with differing width and depth are not supported by Godot Jolt.",
			new_width,
			new_depth,
			height_count
		)
	);

	const auto sample_count = (JPH::uint32)new_width;

	ERR_FAIL_COND_MSG(
		!is_power_of_2(sample_count),
		vformat(
			"Failed to create height map shape with width '{}', depth '{}' and height count '{}'. "
			"Height maps with a width/depth that is not a power of two are not supported by Godot "
			"Jolt.",
			new_width,
			new_depth,
			height_count
		)
	);

	const int width_tiles = new_width - 1;
	const int depth_tiles = new_depth - 1;

	const float half_width_tiles = (float)width_tiles / 2.0f;
	const float half_depth_tiles = (float)depth_tiles / 2.0f;

	const JPH::HeightFieldShapeSettings shape_settings(
		new_heights.ptr(),
		JPH::Vec3(-half_width_tiles, 0, -half_depth_tiles),
		JPH::Vec3::sReplicate(1.0f),
		sample_count
	);

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create height map shape with width '{}', depth '{}' and height count '{}'. "
			"Jolt returned the following error: '{}'.",
			new_width,
			new_depth,
			height_count,
			shape_result.GetError()
		)
	);

	jolt_ref = shape_result.Get();
	heights = new_heights;
	width = new_width;
	depth = new_depth;
}

void JoltHeightMapShape3D::clear_data() {
	jolt_ref = nullptr;
	heights.clear();
	width = 0;
	depth = 0;
}
