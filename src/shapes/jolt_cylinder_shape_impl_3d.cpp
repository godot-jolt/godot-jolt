#include "jolt_cylinder_shape_impl_3d.hpp"

Variant JoltCylinderShapeImpl3D::get_data() const {
	Dictionary data;
	data["height"] = height;
	data["radius"] = radius;
	return data;
}

void JoltCylinderShapeImpl3D::set_data(const Variant& p_data) {
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

void JoltCylinderShapeImpl3D::set_margin(float p_margin) {
	ON_SCOPE_EXIT {
		invalidated();
	};

	destroy();

	margin = p_margin;
}

String JoltCylinderShapeImpl3D::to_string() const {
	return vformat("{height=%f radius=%f margin=%f}", height, radius, margin);
}

JPH::ShapeRefC JoltCylinderShapeImpl3D::build() const {
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
