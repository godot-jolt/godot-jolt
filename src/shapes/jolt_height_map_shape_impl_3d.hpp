#pragma once

#include "shapes/jolt_shape_impl_3d.hpp"

class JoltHeightMapShapeImpl3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_HEIGHTMAP; }

	bool is_convex() const override { return false; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC _build() const override;

	JPH::ShapeRefC _build_height_field() const;

	JPH::ShapeRefC _build_mesh() const;

	JPH::ShapeRefC _build_double_sided(const JPH::Shape* p_shape) const;

	PackedFloat32Array heights;

	int32_t width = 0;

	int32_t depth = 0;
};
