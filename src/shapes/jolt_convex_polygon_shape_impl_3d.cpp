#include "jolt_convex_polygon_shape_impl_3d.hpp"

#include "servers/jolt_project_settings.hpp"

Variant JoltConvexPolygonShapeImpl3D::get_data() const {
	return vertices;
}

void JoltConvexPolygonShapeImpl3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR3_ARRAY);

	vertices = p_data;
}

void JoltConvexPolygonShapeImpl3D::set_margin(float p_margin) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	margin = p_margin;
}

String JoltConvexPolygonShapeImpl3D::to_string() const {
	return vformat("{vertex_count=%d margin=%f}", vertices.size(), margin);
}

JPH::ShapeRefC JoltConvexPolygonShapeImpl3D::build() const {
	const auto vertex_count = (int32_t)vertices.size();

	ERR_FAIL_COND_D_MSG(
		vertex_count < 3,
		vformat(
			"Failed to build convex polygon shape with %s. "
			"It must have a vertex count of at least 3. "
			"This shape belongs to %s.",
			to_string(),
			owners_to_string()
		)
	);

	JPH::Array<JPH::Vec3> jolt_vertices;
	jolt_vertices.reserve((size_t)vertex_count);

	const Vector3* vertices_begin = &vertices[0];
	const Vector3* vertices_end = vertices_begin + vertex_count;

	for (const Vector3* vertex = vertices_begin; vertex != vertices_end; ++vertex) {
		jolt_vertices.emplace_back(vertex->x, vertex->y, vertex->z);
	}

	const float actual_margin = JoltProjectSettings::use_shape_margins() ? margin : 0.0f;

	const JPH::ConvexHullShapeSettings shape_settings(jolt_vertices, actual_margin);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build convex polygon shape with %s. "
			"It returned the following error: '%s'. "
			"This shape belongs to %s.",
			to_string(),
			to_godot(shape_result.GetError()),
			owners_to_string()
		)
	);

	return shape_result.Get();
}
