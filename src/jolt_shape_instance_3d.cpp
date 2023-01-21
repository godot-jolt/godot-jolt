#include "jolt_shape_instance_3d.hpp"

#include "jolt_shape_3d.hpp"

bool JoltShapeInstance3D::try_build(
	const JoltShapeInstance3D& p_shape,
	uint64_t p_user_data,
	Built& p_built_shape
) {
	if (p_shape.is_disabled()) {
		return false;
	}

	JPH::ShapeRefC jolt_ref = p_shape->try_build(p_user_data);

	if (jolt_ref == nullptr) {
		return false;
	}

	p_built_shape.shape = &p_shape;
	p_built_shape.jolt_ref = std::move(jolt_ref);

	return true;
}

int32_t JoltShapeInstance3D::try_build(
	const JoltShapeInstance3D* p_shapes,
	int32_t p_count,
	Built* p_built_shapes
) {
	Built* built_shapes_begin = p_built_shapes;
	Built* built_shapes_end = built_shapes_begin;

	for (int32_t i = 0; i < p_count; ++i) {
		Built built_shape;
		if (try_build(p_shapes[i], (uint64_t)i, built_shape)) {
			*built_shapes_end++ = std::move(built_shape);
		}
	}

	return int32_t(built_shapes_end - built_shapes_begin);
}

JPH::ShapeRefC JoltShapeInstance3D::build_compound(const Built* p_built_shapes, int32_t p_count) {
	JPH::StaticCompoundShapeSettings shape_settings;

	for (int32_t i = 0; i < p_count; ++i) {
		const Built& built_shape = p_built_shapes[i];
		const JoltShapeInstance3D& shape = *built_shape.shape;
		const JPH::ShapeRefC& jolt_ref = built_shape.jolt_ref;

		shape_settings.AddShape(
			to_jolt(shape.get_transform().origin),
			to_jolt(shape.get_transform().basis),
			jolt_ref
		);
	}

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create compound shape with sub-shape count '%d'. "
			"Jolt returned the following error: '%s'.",
			p_count,
			shape_result.GetError().c_str()
		)
	);

	return shape_result.Get();
}
