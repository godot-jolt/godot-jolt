#pragma once

#include "shapes/jolt_custom_shape_type.hpp"

class JoltCustomEmptyShapeSettings final : public JPH::ShapeSettings {
public:
	JoltCustomEmptyShapeSettings() = default;

	ShapeResult Create() const override;
};

class JoltCustomEmptyShape final : public JPH::Shape {
public:
	static void register_type();

	JoltCustomEmptyShape()
		: Shape(JoltCustomShapeType::EMPTY, JoltCustomShapeSubType::EMPTY) { }

	JoltCustomEmptyShape(const JoltCustomEmptyShapeSettings& p_settings, ShapeResult& p_result)
		: Shape(JoltCustomShapeType::EMPTY, JoltCustomShapeSubType::EMPTY, p_settings, p_result) {
		if (!p_result.HasError()) {
			p_result.Set(this);
		}
	}

	JPH::AABox GetLocalBounds() const override { return {JPH::Vec3::sZero(), JPH::Vec3::sZero()}; }

	JPH::uint GetSubShapeIDBitsRecursive() const override { return 0; }

	float GetInnerRadius() const override { return 0.0f; }

	JPH::MassProperties GetMassProperties() const override {
		// HACK(mihe): We need to give the shape a mass, otherwise we'll get an assert when we use
		// this in dynamic or kinematic bodies.
		JPH::MassProperties mass_properties;
		mass_properties.mMass = 1.0f;
		return mass_properties;
	}

	const JPH::PhysicsMaterial* GetMaterial([[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id
	) const override {
		return JPH::PhysicsMaterial::sDefault;
	}

	JPH::Vec3 GetSurfaceNormal(
		[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id,
		[[maybe_unused]] JPH::Vec3Arg p_local_surface_position
	) const override {
		return JPH::Vec3::sZero();
	}

	// clang-format off

	void GetSubmergedVolume(
		[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform,
		[[maybe_unused]] JPH::Vec3Arg p_scale,
		[[maybe_unused]] const JPH::Plane& p_surface,
		float& p_total_volume,
		float& p_submerged_volume,
		JPH::Vec3& p_center_of_buoyancy
#ifdef JPH_DEBUG_RENDERER
		, [[maybe_unused]] JPH::RVec3Arg p_base_offset
#endif // JPH_DEBUG_RENDERER
	) const override {
		p_total_volume = 0.0f;
		p_submerged_volume = 0.0f;
		p_center_of_buoyancy = JPH::Vec3::sZero();
	}

	// clang-format on

#ifdef JPH_DEBUG_RENDERER
	void Draw(
		[[maybe_unused]] JPH::DebugRenderer* p_renderer,
		[[maybe_unused]] JPH::RMat44Arg p_center_of_mass_transform,
		[[maybe_unused]] JPH::Vec3Arg p_scale,
		[[maybe_unused]] JPH::ColorArg p_color,
		[[maybe_unused]] bool p_use_material_colors,
		[[maybe_unused]] bool p_draw_wireframe
	) const override { }
#endif // JPH_DEBUG_RENDERER

	bool CastRay(
		[[maybe_unused]] const JPH::RayCast& p_ray,
		[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator,
		[[maybe_unused]] JPH::RayCastResult& p_hit
	) const override {
		return false;
	}

	void CastRay(
		[[maybe_unused]] const JPH::RayCast& p_ray,
		[[maybe_unused]] const JPH::RayCastSettings& p_ray_cast_settings,
		[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator,
		[[maybe_unused]] JPH::CastRayCollector& p_collector,
		[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter = {}
	) const override { }

	void CollidePoint(
		[[maybe_unused]] JPH::Vec3Arg p_point,
		[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator,
		[[maybe_unused]] JPH::CollidePointCollector& p_collector,
		[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter = {}
	) const override { }

	void CollideSoftBodyVertices(
		[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform,
		[[maybe_unused]] JPH::Vec3Arg p_scale,
		[[maybe_unused]] JPH::SoftBodyVertex* p_vertices,
		[[maybe_unused]] JPH::uint p_num_vertices,
		[[maybe_unused]] float p_delta_time,
		[[maybe_unused]] JPH::Vec3Arg p_displacement_due_to_gravity,
		[[maybe_unused]] int p_colliding_shape_index
	) const override { }

	void GetTrianglesStart(
		[[maybe_unused]] GetTrianglesContext& p_context,
		[[maybe_unused]] const JPH::AABox& p_box,
		[[maybe_unused]] JPH::Vec3Arg p_position_com,
		[[maybe_unused]] JPH::QuatArg p_rotation,
		[[maybe_unused]] JPH::Vec3Arg p_scale
	) const override { }

	int GetTrianglesNext(
		[[maybe_unused]] GetTrianglesContext& p_context,
		[[maybe_unused]] int p_max_triangles_requested,
		[[maybe_unused]] JPH::Float3* p_triangle_vertices,
		[[maybe_unused]] const JPH::PhysicsMaterial** p_materials = nullptr
	) const override {
		return 0;
	}

	Stats GetStats() const override { return {sizeof(*this), 0}; }

	float GetVolume() const override { return 0.0f; }
};
