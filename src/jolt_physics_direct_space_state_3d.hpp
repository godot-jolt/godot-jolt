#pragma once

class JoltSpace3D;

class JoltPhysicsDirectSpaceState3D final : public PhysicsDirectSpaceState3DExtension {
	GDCLASS_NO_WARN(JoltPhysicsDirectSpaceState3D, PhysicsDirectSpaceState3DExtension) // NOLINT

protected:
	// NOLINTNEXTLINE(readability-identifier-naming)
	static void _bind_methods() { }

public:
	JoltPhysicsDirectSpaceState3D() = default;

	explicit JoltPhysicsDirectSpaceState3D(JoltSpace3D* p_space);

	bool _intersect_ray(
		const Vector3& p_from,
		const Vector3& p_to,
		int64_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		bool p_hit_from_inside,
		bool p_hit_back_faces,
		PhysicsServer3DExtensionRayResult* p_result
	) override;

	int64_t _intersect_point(
		const Vector3& p_position,
		int64_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int64_t p_max_results
	) override;

	int64_t _intersect_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		int64_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_result_count,
		int64_t p_max_results
	) override;

	bool _cast_motion(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		int64_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		float* p_closest_safe,
		float* p_closest_unsafe,
		PhysicsServer3DExtensionShapeRestInfo* p_info
	) override;

	bool _collide_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		int64_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		void* p_results,
		int64_t p_max_results,
		int32_t* p_result_count
	) override;

	bool _rest_info(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		int64_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeRestInfo* p_rest_info
	) override;

	Vector3 _get_closest_point_to_object_volume(const RID& p_object, const Vector3& p_point)
		const override;

private:
	JoltSpace3D* space = nullptr;
};
