#include "jolt_shape_3d.hpp"

#include "objects/jolt_collision_object_3d.hpp"
#include "shapes/jolt_override_user_data_shape.hpp"
#include "shapes/jolt_ray_shape.hpp"

JoltShapeImpl3D::~JoltShapeImpl3D() = default;

void JoltShapeImpl3D::add_owner(JoltObjectImpl3D* p_owner) {
	ref_counts_by_owner[p_owner]++;
}

void JoltShapeImpl3D::remove_owner(JoltObjectImpl3D* p_owner) {
	if (--ref_counts_by_owner[p_owner] <= 0) {
		ref_counts_by_owner.erase(p_owner);
	}
}

void JoltShapeImpl3D::remove_self(bool p_lock) {
	// `remove_owner` will be called when we `remove_shape`, so we need to copy the map since the
	// iterator would be invalidated from underneath us
	const auto ref_counts_by_owner_copy = ref_counts_by_owner;

	for (const auto& [owner, ref_count] : ref_counts_by_owner_copy) {
		owner->remove_shape(this, p_lock);
	}
}

JPH::ShapeRefC JoltShapeImpl3D::try_build() {
	if (jolt_ref == nullptr) {
		jolt_ref = build();
	}

	return jolt_ref;
}

JPH::ShapeRefC JoltShapeImpl3D::with_scale(const JPH::Shape* p_shape, const Vector3& p_scale) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::ScaledShapeSettings shape_settings(p_shape, to_jolt(p_scale));
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to scale shape with {scale=%v}. "
			"It returned the following error: '%s'.",
			p_scale,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShapeImpl3D::with_basis_origin(
	const JPH::Shape* p_shape,
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
			"Failed to offset shape with {basis=%s origin=%v}. "
			"It returned the following error: '%s'.",
			p_basis,
			p_origin,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShapeImpl3D::with_transform(
	const JPH::Shape* p_shape,
	const Transform3D& p_transform,
	const Vector3& p_scale
) {
	ERR_FAIL_NULL_D(p_shape);

	JPH::ShapeRefC shape = p_shape;

	if (p_scale != Vector3(1.0f, 1.0f, 1.0f)) {
		shape = with_scale(shape, p_scale);
	}

	if (p_transform != Transform3D()) {
		shape = with_basis_origin(shape, p_transform.basis, p_transform.origin);
	}

	return shape;
}

JPH::ShapeRefC JoltShapeImpl3D::with_center_of_mass_offset(
	const JPH::Shape* p_shape,
	const Vector3& p_offset
) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::OffsetCenterOfMassShapeSettings shape_settings(to_jolt(p_offset), p_shape);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to offset center of mass with {offset=%v}. "
			"It returned the following error: '%s'.",
			p_offset,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShapeImpl3D::with_center_of_mass(
	const JPH::Shape* p_shape,
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

JPH::ShapeRefC JoltShapeImpl3D::with_user_data(const JPH::Shape* p_shape, uint64_t p_user_data) {
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

void JoltShapeImpl3D::invalidated(bool p_lock) {
	for (const auto& [owner, ref_count] : ref_counts_by_owner) {
		owner->shapes_changed(p_lock);
	}
}

Variant JoltWorldBoundaryShape3D::get_data() const {
	return plane;
}

void JoltWorldBoundaryShape3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::PLANE);

	plane = p_data;
}

JPH::ShapeRefC JoltWorldBoundaryShape3D::build() const {
	ERR_FAIL_D_MSG(
		"WorldBoundaryShape3D is not supported by Godot Jolt. "
		"Consider using one or more reasonably sized BoxShape3D instead."
	);
}

Variant JoltSeparationRayShape3D::get_data() const {
	Dictionary data;
	data["length"] = length;
	data["slide_on_slope"] = slide_on_slope;
	return data;
}

void JoltSeparationRayShape3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_length = data.get("length", {});
	ERR_FAIL_COND(maybe_length.get_type() != Variant::FLOAT);

	const Variant maybe_slide_on_slope = data.get("slide_on_slope", {});
	ERR_FAIL_COND(maybe_slide_on_slope.get_type() != Variant::BOOL);

	length = maybe_length;
	slide_on_slope = maybe_slide_on_slope;
}

String JoltSeparationRayShape3D::to_string() const {
	return vformat("{length=%f slide_on_slope=%s}", length, slide_on_slope);
}

JPH::ShapeRefC JoltSeparationRayShape3D::build() const {
	ERR_FAIL_COND_D_MSG(
		length <= 0.0f,
		vformat(
			"Failed to build separation ray shape with %s. "
			"Its length must be greater than 0.",
			to_string()
		)
	);

	const JoltRayShapeSettings shape_settings(length, slide_on_slope);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build separation ray shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltSphereShape3D::get_data() const {
	return radius;
}

void JoltSphereShape3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::FLOAT);

	radius = p_data;
}

String JoltSphereShape3D::to_string() const {
	return vformat("{radius=%f}", radius);
}

JPH::ShapeRefC JoltSphereShape3D::build() const {
	ERR_FAIL_COND_D_MSG(
		radius <= 0.0f,
		vformat(
			"Failed to build sphere shape with %s. "
			"Its radius must be greater than 0.",
			to_string()
		)
	);

	const JPH::SphereShapeSettings shape_settings(radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build sphere shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltBoxShape3D::get_data() const {
	return half_extents;
}

void JoltBoxShape3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR3);

	half_extents = p_data;
}

void JoltBoxShape3D::set_margin(float p_margin) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	margin = p_margin;
}

String JoltBoxShape3D::to_string() const {
	return vformat("{half_extents=%v margin=%f}", half_extents, margin);
}

JPH::ShapeRefC JoltBoxShape3D::build() const {
	const float shortest_axis = half_extents[half_extents.min_axis_index()];

	ERR_FAIL_COND_D_MSG(
		shortest_axis <= margin,
		vformat(
			"Failed to build box shape with %s. "
			"Its half extents must be greater than its margin.",
			to_string()
		)
	);

	const JPH::BoxShapeSettings shape_settings(to_jolt(half_extents), margin);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build box shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
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
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_height = data.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = data.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	height = maybe_height;
	radius = maybe_radius;
}

String JoltCapsuleShape3D::to_string() const {
	return vformat("{height=%f radius=%f}", height, radius);
}

JPH::ShapeRefC JoltCapsuleShape3D::build() const {
	ERR_FAIL_COND_D_MSG(
		radius <= 0.0f,
		vformat(
			"Failed to build capsule shape with %s. "
			"Its radius must be greater than 0.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		height <= 0.0f,
		vformat(
			"Failed to build capsule shape with %s. "
			"Its height must be greater than 0.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		height < radius * 2.0f,
		vformat(
			"Failed to build capsule shape with %s. "
			"Its height must be at least double that of its radius.",
			to_string()
		)
	);

	const float half_height = height / 2.0f;
	const float cylinder_height = half_height - radius;

	const JPH::CapsuleShapeSettings shape_settings(cylinder_height, radius);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build capsule shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
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
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_height = data.get("height", {});
	ERR_FAIL_COND(maybe_height.get_type() != Variant::FLOAT);

	const Variant maybe_radius = data.get("radius", {});
	ERR_FAIL_COND(maybe_radius.get_type() != Variant::FLOAT);

	height = maybe_height;
	radius = maybe_radius;
}

void JoltCylinderShape3D::set_margin(float p_margin) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	margin = p_margin;
}

String JoltCylinderShape3D::to_string() const {
	return vformat("{height=%f radius=%f margin=%f}", height, radius, margin);
}

JPH::ShapeRefC JoltCylinderShape3D::build() const {
	ERR_FAIL_COND_D_MSG(
		height < margin * 2.0f,
		vformat(
			"Failed to build cylinder shape with %s. "
			"Its height must be at least double that of its margin.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		radius < margin,
		vformat(
			"Failed to build cylinder shape with %s. "
			"Its radius must be equal to or greater than its margin.",
			to_string()
		)
	);

	const float half_height = height / 2.0f;

	const JPH::CylinderShapeSettings shape_settings(half_height, radius, margin);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build cylinder shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

Variant JoltConvexPolygonShape3D::get_data() const {
	return vertices;
}

void JoltConvexPolygonShape3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	vertices = p_data;
}

void JoltConvexPolygonShape3D::set_margin(float p_margin) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	margin = p_margin;
}

String JoltConvexPolygonShape3D::to_string() const {
	return vformat("{vertex_count=%d margin=%f}", vertices.size(), margin);
}

JPH::ShapeRefC JoltConvexPolygonShape3D::build() const {
	const auto vertex_count = (int32_t)vertices.size();

	ERR_FAIL_COND_D_MSG(
		vertex_count < 3,
		vformat(
			"Failed to build convex polygon shape with %s. "
			"It must have a vertex count of at least 3.",
			to_string()
		)
	);

	JPH::Array<JPH::Vec3> jolt_vertices;
	jolt_vertices.reserve((size_t)vertex_count);

	const Vector3* vertices_begin = &vertices[0];
	const Vector3* vertices_end = vertices_begin + vertex_count;

	for (const Vector3* vertex = vertices_begin; vertex != vertices_end; ++vertex) {
		jolt_vertices.emplace_back(vertex->x, vertex->y, vertex->z);
	}

	const JPH::ConvexHullShapeSettings shape_settings(jolt_vertices, margin);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build convex polygon shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
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
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_faces = data.get("faces", {});
	ERR_FAIL_COND(maybe_faces.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	const Variant maybe_backface_collision = data.get("backface_collision", {});
	ERR_FAIL_COND(maybe_backface_collision.get_type() != Variant::BOOL);

	faces = maybe_faces;
	backface_collision = maybe_backface_collision;
}

String JoltConcavePolygonShape3D::to_string() const {
	return vformat("{vertex_count=%d}", faces.size());
}

JPH::ShapeRefC JoltConcavePolygonShape3D::build() const {
	const auto vertex_count = (int32_t)faces.size();
	const int32_t face_count = vertex_count / 3;
	const int32_t excess_vertex_count = vertex_count % 3;

	// HACK(mihe): We can't emit an error for a vertex count of 0 because of things like CSGShape3D
	// which has its proper initialization deferred, leading to errors being emitted for every
	// single one that's created
	QUIET_FAIL_COND_D(vertex_count == 0);

	ERR_FAIL_COND_D_MSG(
		vertex_count < 3,
		vformat(
			"Failed to build concave polygon shape with %s. "
			"It must have a vertex count of at least 3.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		excess_vertex_count != 0,
		vformat(
			"Failed to build concave polygon shape with %s. "
			"It must have a vertex count that is divisible by 3.",
			to_string()
		)
	);

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
			"Failed to build concave polygon shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
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
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_heights = data.get("heights", {});
	ERR_FAIL_COND(maybe_heights.get_type() != Variant::PACKED_FLOAT32_ARRAY);

	const Variant maybe_width = data.get("width", {});
	ERR_FAIL_COND(maybe_width.get_type() != Variant::INT);

	const Variant maybe_depth = data.get("depth", {});
	ERR_FAIL_COND(maybe_depth.get_type() != Variant::INT);

	heights = maybe_heights;
	width = maybe_width;
	depth = maybe_depth;
}

String JoltHeightMapShape3D::to_string() const {
	return vformat("{height_count=%d width=%d depth=%d}", heights.size(), width, depth);
}

JPH::ShapeRefC JoltHeightMapShape3D::build() const {
	const auto height_count = (int32_t)heights.size();

	// NOTE(mihe): This somewhat arbitrary limit depends on what the block size is set to, which by
	// default is 2. If it's set any higher then so would this limit need to be.
	ERR_FAIL_COND_D_MSG(
		height_count < 16,
		vformat(
			"Failed to build height map shape with %s. "
			"Height count must be at least 16.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		height_count != width * depth,
		vformat(
			"Failed to build height map shape with %s. "
			"Height count must be the product of width and depth.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		width != depth,
		vformat(
			"Failed to build height map shape with %s. "
			"Width must be equal to depth.",
			to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		!is_power_of_2((uint32_t)width),
		vformat(
			"Failed to build height map shape with %s. "
			"Width/depth must be a power-of-two.",
			to_string()
		)
	);

	const int32_t segments_width = width - 1;
	const int32_t segments_height = depth - 1;

	const float segments_half_width = (float)segments_width / 2.0f;
	const float segments_half_depth = (float)segments_height / 2.0f;

	const JPH::HeightFieldShapeSettings shape_settings(
		heights.ptr(),
		JPH::Vec3(-segments_half_width, 0, -segments_half_depth),
		JPH::Vec3::sReplicate(1.0f),
		(JPH::uint32)width
	);

	// TOOD(mihe): Calculate the necessary bits per sample using `CalculateBitsPerSampleForError`

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build height map shape with %s. "
			"It returned the following error: '%s'.",
			to_string(),
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}
