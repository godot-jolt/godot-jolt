#include "jolt_shape_3d.hpp"

#include "jolt_collision_object_3d.hpp"
#include "jolt_override_user_data_shape.hpp"

namespace {

constexpr float GDJOLT_CONVEX_RADIUS = 0.0f;

} // namespace

JoltShape3D::~JoltShape3D() = default;

void JoltShape3D::add_owner(JoltCollisionObject3D* p_owner) {
	ref_counts_by_owner[p_owner]++;
}

void JoltShape3D::remove_owner(JoltCollisionObject3D* p_owner) {
	if (--ref_counts_by_owner[p_owner] <= 0) {
		ref_counts_by_owner.erase(p_owner);
	}
}

void JoltShape3D::remove_self(bool p_lock) {
	// `remove_owner` will be called when we `remove_shape`, so we need to copy the map since the
	// iterator would be invalidated from underneath us
	const auto ref_counts_by_owner_copy = ref_counts_by_owner;

	for (const auto& [owner, ref_count] : ref_counts_by_owner_copy) {
		owner->remove_shape(this, p_lock);
	}
}

JPH::ShapeRefC JoltShape3D::try_build() {
	if (!is_valid()) {
		return {};
	}

	if (jolt_ref == nullptr) {
		jolt_ref = build();
	}

	return jolt_ref;
}

JPH::ShapeRefC JoltShape3D::with_scale(const JPH::ShapeRefC& p_shape, const Vector3& p_scale) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::ScaledShapeSettings shape_settings(p_shape, to_jolt(p_scale));
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to scale shape with scale '%v'. "
			"Jolt returned the following error: '%s'.",
			p_scale,
			to_godot(shape_result.GetError())
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
			"Failed to offset shape with basis '%s' and origin '%v'. "
			"Jolt returned the following error: '%s'.",
			p_basis,
			p_origin,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShape3D::with_transform(
	const JPH::ShapeRefC& p_shape,
	const Transform3D& p_transform
) {
	ERR_FAIL_NULL_D(p_shape);

	JPH::ShapeRefC shape = p_shape;
	Transform3D transform = p_transform;

	if (transform == Transform3D()) {
		return shape;
	}

	const Vector3 scale_squared(
		transform.basis.get_column(Vector3::AXIS_X).length_squared(),
		transform.basis.get_column(Vector3::AXIS_Y).length_squared(),
		transform.basis.get_column(Vector3::AXIS_Z).length_squared()
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
			"Failed to offset center of mass with offset '%v'. "
			"Jolt returned the following error: '%s'.",
			p_offset,
			to_godot(shape_result.GetError())
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

JPH::ShapeRefC JoltShape3D::with_user_data(const JPH::ShapeRefC& p_shape, uint64_t p_user_data) {
	JoltOverrideUserDataShapeSettings shape_settings(p_shape);
	shape_settings.mUserData = (JPH::uint64)p_user_data;

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to override user data. "
			"It returned the following error: '%s'.",
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltSphereShape3D::get_data() const {
	return radius;
}

void JoltSphereShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::FLOAT);

	const float new_radius = p_data;

	// Godot seems to be forgiving about zero-sized shapes, so we try to mimick that by silently
	// letting these remain invalid.
	if (new_radius <= 0.0f) {
		return;
	}

	radius = new_radius;
}

void JoltSphereShape3D::clear() {
	jolt_ref = nullptr;
	radius = 0.0f;
}

JPH::ShapeRefC JoltSphereShape3D::build() const {
	const JPH::SphereShapeSettings shape_settings(radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build sphere shape with radius '%f'. "
			"Jolt returned the following error: '%s'.",
			radius,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltBoxShape3D::get_data() const {
	return half_extents;
}

void JoltBoxShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR3);

	const Vector3 new_half_extents = p_data;
	const float shortest_axis = new_half_extents[new_half_extents.min_axis_index()];

	// Godot seems to be forgiving about zero-sized shapes, so we try to mimick that by silently
	// letting these remain invalid. We also treat any extents smaller than or equal to the convex
	// radius as zero-sized, otherwise Jolt will report an error.
	if (shortest_axis <= GDJOLT_CONVEX_RADIUS) {
		return;
	}

	half_extents = new_half_extents;
}

void JoltBoxShape3D::clear() {
	jolt_ref = nullptr;
	half_extents.zero();
}

JPH::ShapeRefC JoltBoxShape3D::build() const {
	const JPH::BoxShapeSettings shape_settings(to_jolt(half_extents), GDJOLT_CONVEX_RADIUS);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build box shape with half extents '%v'. "
			"Jolt returned the following error: '%s'.",
			half_extents,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltCapsuleShape3D::get_data() const {
	Dictionary data;
	data["height"] = height;
	data["radius"] = radius;
	return data;
}

void JoltCapsuleShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_height = data.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = data.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	const float new_height = maybe_height;
	const float new_radius = maybe_radius;

	// Godot seems to be forgiving about zero-sized shapes, so we try to mimick that by silently
	// letting these remain invalid.
	if (new_height <= 0.0f || new_radius <= 0.0f) {
		return;
	}

	const float half_height = new_height / 2.0f;

	ERR_FAIL_COND_MSG(
		half_height < new_radius,
		vformat(
			"Failed to set shape data for capsule shape with height '%f' and radius '%f'. "
			"Half height must be equal to or greater than radius.",
			new_height,
			new_radius
		)
	);

	height = new_height;
	radius = new_radius;
}

void JoltCapsuleShape3D::clear() {
	jolt_ref = nullptr;
	height = 0.0f;
	radius = 0.0f;
}

JPH::ShapeRefC JoltCapsuleShape3D::build() const {
	const float half_height = height / 2.0f;
	const float clamped_height = max(half_height - radius, CMP_EPSILON);

	const JPH::CapsuleShapeSettings shape_settings(clamped_height, radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build capsule shape with height '%f' and radius '%f'. "
			"Jolt returned the following error: '%s'.",
			height,
			radius,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltCylinderShape3D::get_data() const {
	Dictionary data;
	data["height"] = height;
	data["radius"] = radius;
	return data;
}

void JoltCylinderShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_height = data.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = data.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	const float new_height = maybe_height;
	const float new_radius = maybe_radius;

	// Godot seems to be forgiving about zero-sized shapes, so we try to mimick that by silently
	// letting these remain invalid. We also treat any extents smaller than the convex radius as
	// zero-sized, otherwise Jolt will report an error.
	if (new_height < GDJOLT_CONVEX_RADIUS || new_radius < GDJOLT_CONVEX_RADIUS) {
		return;
	}

	height = new_height;
	radius = new_radius;
}

void JoltCylinderShape3D::clear() {
	jolt_ref = nullptr;
	height = 0.0f;
	radius = 0.0f;
}

JPH::ShapeRefC JoltCylinderShape3D::build() const {
	const float half_height = height / 2.0f;

	const JPH::CylinderShapeSettings shape_settings(half_height, radius, GDJOLT_CONVEX_RADIUS);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build cylinder shape with height '%f' and radius '%f'. "
			"Jolt returned the following error: '%s'.",
			height,
			radius,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltConvexPolygonShape3D::get_data() const {
	return vertices;
}

void JoltConvexPolygonShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	PackedVector3Array new_vertices = p_data;
	const int64_t vertex_count = new_vertices.size();

	// Godot seems to be forgiving about zero-sized shapes, so we try to mimick that by silently
	// letting these remain invalid.
	if (vertex_count < 3) {
		return;
	}

	vertices = std::move(new_vertices);
}

void JoltConvexPolygonShape3D::clear() {
	jolt_ref = nullptr;
	vertices.clear();
}

JPH::ShapeRefC JoltConvexPolygonShape3D::build() const {
	const auto vertex_count = (int32_t)vertices.size();

	JPH::Array<JPH::Vec3> jolt_vertices;
	jolt_vertices.reserve((size_t)vertex_count);

	const Vector3* vertices_begin = &vertices[0];
	const Vector3* vertices_end = vertices_begin + vertex_count;

	for (const Vector3* vertex = vertices_begin; vertex != vertices_end; ++vertex) {
		jolt_vertices.emplace_back(vertex->x, vertex->y, vertex->z);
	}

	const JPH::ConvexHullShapeSettings shape_settings(jolt_vertices, GDJOLT_CONVEX_RADIUS);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build convex polygon shape with vertex count '%d'. "
			"Jolt returned the following error: '%s'.",
			vertex_count,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltConcavePolygonShape3D::get_data() const {
	Dictionary data;
	data["faces"] = faces;
	data["backface_collision"] = backface_collision;
	return data;
}

void JoltConcavePolygonShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_faces = data.get("faces", {});
	ERR_FAIL_COND(maybe_faces.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	const Variant maybe_backface_collision = data.get("backface_collision", {});
	ERR_FAIL_COND(maybe_backface_collision.get_type() != Variant::BOOL);

	PackedVector3Array new_faces = maybe_faces;
	const bool new_backface_collision = maybe_backface_collision;

	const auto vertex_count = (size_t)new_faces.size();

	// Godot seems to be forgiving about zero-sized shapes, so we try to mimick that by silently
	// letting these remain invalid.
	if (vertex_count == 0) {
		return;
	}

	const size_t excess_vertex_count = vertex_count % 3;

	ERR_FAIL_COND_MSG(
		excess_vertex_count != 0,
		"Failed to set shape data for concave polygon shape with vertex count '{}'. "
		"Expected a vertex count divisible by 3."
	);

	const size_t face_count = vertex_count / 3;

	if (face_count == 0) {
		return;
	}

	faces = std::move(new_faces);
	backface_collision = new_backface_collision;
}

void JoltConcavePolygonShape3D::clear() {
	jolt_ref = nullptr;
	faces.clear();
	backface_collision = false;
}

JPH::ShapeRefC JoltConcavePolygonShape3D::build() const {
	const auto vertex_count = (int32_t)faces.size();
	const int32_t face_count = vertex_count / 3;

	JPH::TriangleList jolt_faces;
	jolt_faces.reserve((size_t)face_count);

	const Vector3* faces_begin = &faces[0];
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

		if (backface_collision) {
			jolt_faces.emplace_back(
				JPH::Float3(v0->x, v0->y, v0->z),
				JPH::Float3(v1->x, v1->y, v1->z),
				JPH::Float3(v2->x, v2->y, v2->z)
			);
		}
	}

	const JPH::MeshShapeSettings shape_settings(jolt_faces);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build concave polygon shape with vertex count '%d'. "
			"Jolt returned the following error: '%s'.",
			vertex_count,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltHeightMapShape3D::get_data() const {
	Dictionary data;
	data["width"] = width;
	data["depth"] = depth;
	data["heights"] = heights;
	return data;
}

void JoltHeightMapShape3D::set_data(const Variant& p_data) {
	clear();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_heights = data.get("heights", {});
	ERR_FAIL_COND(maybe_heights.get_type() != Variant::PACKED_FLOAT32_ARRAY);

	const Variant maybe_width = data.get("width", {});
	ERR_FAIL_COND(maybe_width.get_type() != Variant::INT);

	const Variant maybe_depth = data.get("depth", {});
	ERR_FAIL_COND(maybe_depth.get_type() != Variant::INT);

	const PackedFloat32Array new_heights = maybe_heights;
	const int32_t new_width = maybe_width;
	const int32_t new_depth = maybe_depth;

	const auto height_count = (int32_t)new_heights.size();

	if (height_count == 0) {
		return;
	}

	// HACK(mihe): A height map shape will have a width or depth of 2 while it's transitioning from
	// its default state. Since Jolt doesn't support non-square height maps, and it's unlikely that
	// anyone would actually want a height map of such small dimensions, we silently let this remain
	// invalid in order to not display an error every single time we create a shape of this type.
	if (new_width <= 2 || new_depth <= 2) {
		return;
	}

	ERR_FAIL_COND_MSG(
		height_count != new_width * new_depth,
		vformat(
			"Failed to set shape data for height map shape with width '%d', depth '%d' and height "
			"count '%d'. Height count must be equal to width multiplied by depth.",
			new_width,
			new_depth,
			height_count
		)
	);

	ERR_FAIL_COND_MSG(
		new_width != new_depth,
		vformat(
			"Failed to set shape data for height map shape with width '%d', depth '%d' and height "
			"count '%d'. Height maps with differing width and depth are not supported by Godot "
			"Jolt.",
			new_width,
			new_depth,
			height_count
		)
	);

	const auto sample_count = (JPH::uint32)new_width;

	ERR_FAIL_COND_MSG(
		!is_power_of_2(sample_count),
		vformat(
			"Failed to set shape data for height map shape with width '%d', depth '%d' and height "
			"count '%d'. Height maps with a width/depth that is not a power of two are not "
			"supported by Godot Jolt.",
			new_width,
			new_depth,
			height_count
		)
	);

	heights = new_heights;
	width = new_width;
	depth = new_depth;
}

void JoltHeightMapShape3D::clear() {
	jolt_ref = nullptr;
	heights.clear();
	width = 0;
	depth = 0;
}

JPH::ShapeRefC JoltHeightMapShape3D::build() const {
	const int32_t width_tiles = width - 1;
	const int32_t depth_tiles = depth - 1;

	const float half_width_tiles = (float)width_tiles / 2.0f;
	const float half_depth_tiles = (float)depth_tiles / 2.0f;

	const JPH::HeightFieldShapeSettings shape_settings(
		heights.ptr(),
		JPH::Vec3(-half_width_tiles, 0, -half_depth_tiles),
		JPH::Vec3::sReplicate(1.0f),
		(JPH::uint32)width
	);

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build height map shape with width '%d', depth '%d' and height count '%d'. "
			"Jolt returned the following error: '%s'.",
			width,
			depth,
			heights.size(),
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}
