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

void JoltShapeInstance3D::try_build(
	const Vector<JoltShapeInstance3D>& p_shapes,
	Vector<Built>& p_built_shapes
) {
	p_built_shapes.clear();
	p_built_shapes.resize(p_shapes.size());

	Built* built_shapes_begin = p_built_shapes.ptrw();
	Built* built_shapes_end = built_shapes_begin;

	for (int shape_idx = 0; shape_idx < p_shapes.size(); ++shape_idx) {
		Built built_shape;
		if (try_build(p_shapes[shape_idx], (uint64_t)shape_idx, built_shape)) {
			*built_shapes_end++ = std::move(built_shape);
		}
	}

	const auto built_shape_count = int(built_shapes_end - built_shapes_begin);

	p_built_shapes.resize(built_shape_count);
}

JPH::ShapeRefC JoltShapeInstance3D::build_compound(const Vector<Built>& p_built_shapes) {
	JPH::StaticCompoundShapeSettings shape_settings;

	for (const Built& built_shape : p_built_shapes) {
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
			"Failed to create compound shape with sub-shape count '{}'. "
			"Jolt returned the following error: '{}'.",
			p_built_shapes.size(),
			shape_result.GetError()
		)
	);

	return shape_result.Get();
}
