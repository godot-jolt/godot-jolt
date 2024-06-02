#pragma once

class JoltBodyImpl3D;
class JoltShapeImpl3D;
class JoltSpace3D;

class JoltPhysicsDirectSpaceState3D final : public PhysicsDirectSpaceState3DExtension {
	GDCLASS_QUIET(JoltPhysicsDirectSpaceState3D, PhysicsDirectSpaceState3DExtension)

private:
	static void _bind_methods() { }

public:
	JoltPhysicsDirectSpaceState3D() = default;

	explicit JoltPhysicsDirectSpaceState3D(JoltSpace3D* p_space);

	bool _intersect_ray(
		const Vector3& p_from,
		const Vector3& p_to,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		bool p_hit_from_inside,
		bool p_hit_back_faces,
		bool p_pick_ray,
		PhysicsServer3DExtensionRayResult* p_result
	) override;

	int32_t _intersect_point(
		const Vector3& p_position,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int32_t p_max_results
	) override;

	int32_t _intersect_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int32_t p_max_results
	) override;

	bool _cast_motion(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		real_t* p_closest_safe,
		real_t* p_closest_unsafe,
		PhysicsServer3DExtensionShapeRestInfo* p_info
	) override;

	bool _collide_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		void* p_results,
		int32_t p_max_results,
		int32_t* p_result_count
	) override;

	bool _rest_info(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeRestInfo* p_info
	) override;

	Vector3 _get_closest_point_to_object_volume(const RID& p_object, const Vector3& p_point)
		const override;

	bool test_body_motion(
		const JoltBodyImpl3D& p_body,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		float p_margin,
		int32_t p_max_collisions,
		bool p_collide_separation_ray,
		bool p_recovery_as_collision,
		PhysicsServer3DExtensionMotionResult* p_result
	) const;

	JoltSpace3D& get_space() const { return *space; }

private:
	bool _cast_motion_impl(
		const JPH::Shape& p_jolt_shape,
		const Transform3D& p_transform_com,
		const Vector3& p_scale,
		const Vector3& p_motion,
		bool p_ignore_overlaps,
		const JPH::CollideShapeSettings& p_settings,
		const JPH::BroadPhaseLayerFilter& p_broad_phase_layer_filter,
		const JPH::ObjectLayerFilter& p_object_layer_filter,
		const JPH::BodyFilter& p_body_filter,
		const JPH::ShapeFilter& p_shape_filter,
		real_t& p_closest_safe,
		real_t& p_closest_unsafe
	) const;

	bool _body_motion_recover(
		const JoltBodyImpl3D& p_body,
		const Transform3D& p_transform,
		float p_margin,
		Vector3& p_recovery
	) const;

	bool _body_motion_cast(
		const JoltBodyImpl3D& p_body,
		const Transform3D& p_transform,
		const Vector3& p_scale,
		const Vector3& p_motion,
		bool p_collide_separation_ray,
		real_t& p_safe_fraction,
		real_t& p_unsafe_fraction
	) const;

	bool _body_motion_collide(
		const JoltBodyImpl3D& p_body,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		float p_margin,
		int32_t p_max_collisions,
		PhysicsServer3DExtensionMotionResult* p_result
	) const;

	void _generate_manifold(
		const JPH::CollideShapeResult& p_hit,
		JPH::ContactPoints& p_contact_points1,
		JPH::ContactPoints& p_contact_points2
#ifdef JPH_DEBUG_RENDERER
		,
		JPH::RVec3Arg p_center_of_mass
#endif // JPH_DEBUG_RENDERER
	) const;

	JoltSpace3D* space = nullptr;
};
