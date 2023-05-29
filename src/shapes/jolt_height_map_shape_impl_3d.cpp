#include "jolt_height_map_shape_impl_3d.hpp"

Variant JoltHeightMapShapeImpl3D::get_data() const {
	Dictionary data;
	data["width"] = width;
	data["depth"] = depth;
	data["heights"] = heights;
	return data;
}

void JoltHeightMapShapeImpl3D::set_data(const Variant& p_data) {
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

String JoltHeightMapShapeImpl3D::to_string() const {
	return vformat("{height_count=%d width=%d depth=%d}", heights.size(), width, depth);
}

JPH::ShapeRefC JoltHeightMapShapeImpl3D::build() const {
	const auto height_count = (int32_t)heights.size();

	// NOTE(mihe): This somewhat arbitrary limit depends on what the block size is set to, which by
	// default is 2. If it's set any higher then so would this limit need to be.
	ERR_FAIL_COND_D_MSG(
		height_count < 16,
		vformat(
			"Failed to build height map shape with %s. "
			"Height count must be at least 16. "
			"This shape belongs to %s.",
			to_string(),
			owners_to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		height_count != width * depth,
		vformat(
			"Failed to build height map shape with %s. "
			"Height count must be the product of width and depth. "
			"This shape belongs to %s.",
			to_string(),
			owners_to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		width != depth,
		vformat(
			"Failed to build height map shape with %s. "
			"Width must be equal to depth. "
			"This shape belongs to %s.",
			to_string(),
			owners_to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		!is_power_of_2((uint32_t)width),
		vformat(
			"Failed to build height map shape with %s. "
			"Width/depth must be a power-of-two. "
			"This shape belongs to %s.",
			to_string(),
			owners_to_string()
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
			"It returned the following error: '%s'. "
			"This shape belongs to %s.",
			to_string(),
			to_godot(shape_result.GetError()),
			owners_to_string()
		)
	);

	return shape_result.Get();
}
