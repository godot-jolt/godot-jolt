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
		_invalidated();
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

JPH::ShapeRefC JoltHeightMapShapeImpl3D::_build() const {
	const auto height_count = (int32_t)heights.size();

	QUIET_FAIL_COND_D(height_count == 0);

	ERR_FAIL_COND_D_MSG(
		height_count != width * depth,
		vformat(
			"Failed to build height map shape with %s. "
			"Height count must be the product of width and depth. "
			"This shape belongs to %s.",
			to_string(),
			_owners_to_string()
		)
	);

	ERR_FAIL_COND_D_MSG(
		width < 2 || depth < 2,
		vformat(
			"Failed to build height map shape with %s. "
			"The height map must be at least 2x2. "
			"This shape belongs to %s.",
			to_string(),
			_owners_to_string()
		)
	);

	if (width != depth) {
		return _build_mesh();
	}

	const int32_t block_size = 2; // Default of JPH::HeightFieldShapeSettings::mBlockSize
	const int32_t block_count = width / block_size;

	if (block_count < 2) {
		return _build_mesh();
	}

	return _build_height_field();
}

JPH::ShapeRefC JoltHeightMapShapeImpl3D::_build_height_field() const {
	const int32_t quad_count_x = width - 1;
	const int32_t quad_count_y = depth - 1;

	const float offset_x = (float)-quad_count_x / 2.0f;
	const float offset_y = (float)-quad_count_y / 2.0f;

	JPH::HeightFieldShapeSettings shape_settings(
		heights.ptr(),
		JPH::Vec3(offset_x, 0, offset_y),
		JPH::Vec3::sReplicate(1.0f),
		(JPH::uint32)width
	);

	shape_settings.mBitsPerSample = shape_settings.CalculateBitsPerSampleForError(0.0f);

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build height map shape with %s. "
			"It returned the following error: '%s'. "
			"This shape belongs to %s.",
			to_string(),
			to_godot(shape_result.GetError()),
			_owners_to_string()
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltHeightMapShapeImpl3D::_build_mesh() const {
	const auto height_count = (int32_t)heights.size();

	const int32_t quad_count_x = width - 1;
	const int32_t quad_count_z = depth - 1;

	const int32_t quad_count = quad_count_x * quad_count_z;
	const int32_t triangle_count = quad_count * 2;

	JPH::VertexList vertices;
	vertices.reserve((size_t)height_count);

	JPH::IndexedTriangleList indices;
	indices.reserve((size_t)triangle_count);

	const float offset_x = (float)-quad_count_x / 2.0f;
	const float offset_z = (float)-quad_count_z / 2.0f;

	for (int32_t z = 0; z < depth; ++z) {
		for (int32_t x = 0; x < width; ++x) {
			const float vertex_x = offset_x + (float)x;
			const float vertex_y = heights[z * width + x];
			const float vertex_z = offset_z + (float)z;

			vertices.emplace_back(vertex_x, vertex_y, vertex_z);
		}
	}

	auto to_index = [&](int32_t p_x, int32_t p_z) {
		return (p_z * width) + p_x;
	};

	for (int32_t z = 0; z < quad_count_z; ++z) {
		for (int32_t x = 0; x < quad_count_x; ++x) {
			const int32_t lr = to_index(x + 0, z + 0);
			const int32_t ll = to_index(x + 1, z + 0);
			const int32_t ur = to_index(x + 0, z + 1);
			const int32_t ul = to_index(x + 1, z + 1);

			indices.emplace_back(lr, ur, ll);
			indices.emplace_back(ll, ur, ul);
		}
	}

	const JPH::MeshShapeSettings shape_settings(std::move(vertices), std::move(indices));
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to build height map shape (as polygon) with %s. "
			"It returned the following error: '%s'. "
			"This shape belongs to %s.",
			to_string(),
			to_godot(shape_result.GetError()),
			_owners_to_string()
		)
	);

	return shape_result.Get();
}
