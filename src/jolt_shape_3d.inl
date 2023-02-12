#pragma once

template<typename TCallable>
JPH::ShapeRefC JoltShape3D::as_compound(TCallable&& p_callable) {
	JPH::StaticCompoundShapeSettings shape_settings;

	auto add_shape = [&](const JPH::ShapeRefC& p_shape, const Transform3D& p_transform) {
		shape_settings.AddShape(to_jolt(p_transform.origin), to_jolt(p_transform.basis), p_shape);
	};

	while (p_callable(add_shape)) {
		// ...
	}

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to create compound shape with sub-shape count '%d'. "
			"Jolt returned the following error: '%s'.",
			(int32_t)shape_settings.mSubShapes.size(),
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}
