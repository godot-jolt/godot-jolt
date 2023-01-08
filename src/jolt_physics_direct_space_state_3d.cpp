#include "jolt_physics_direct_space_state_3d.hpp"

#include "jolt_body_access_3d.hpp"
#include "jolt_broad_phase_layer.hpp"
#include "jolt_collision_object_3d.hpp"
#include "jolt_object_layer.hpp"
#include "jolt_space_3d.hpp"

class JoltQueryBroadPhaseLayerFilter3D : public JPH::BroadPhaseLayerFilter {
	bool ShouldCollide(JPH::BroadPhaseLayer p_layer) const override {
		return p_layer != JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_NONE);
	}
};

class JoltQueryObjectLayerFilter3D : public JPH::ObjectLayerFilter {
	bool ShouldCollide(JPH::ObjectLayer p_layer) const override {
		return p_layer != JPH::ObjectLayer(GDJOLT_OBJECT_LAYER_NONE);
	}
};

class JoltQueryBodyFilter3D : public JPH::BodyFilter {
public:
	JoltQueryBodyFilter3D(
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas
	)
		: collision_mask(p_collision_mask)
		, collide_with_bodies(p_collide_with_bodies)
		, collide_with_areas(p_collide_with_areas)
		, collision_group(
			  nullptr,
			  (JPH::CollisionGroup::GroupID)0,
			  (JPH::CollisionGroup::SubGroupID)collision_mask
		  ) { }

	bool ShouldCollideLocked(const JPH::Body& p_body) const override {
		if (!collide_with_bodies && !p_body.IsSensor()) {
			return false;
		}

		if (!collide_with_areas && p_body.IsSensor()) {
			return false;
		}

		return collision_group.CanCollide(p_body.GetCollisionGroup());
	}

private:
	uint32_t collision_mask = 0;

	bool collide_with_bodies = false;

	bool collide_with_areas = false;

	JPH::CollisionGroup collision_group;
};

class JoltQueryShapeFilter3D : public JPH::ShapeFilter {
	bool ShouldCollide([[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id_2) const override {
		return true;
	}

	bool ShouldCollide(
		[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id_1,
		[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id_2
	) const override {
		return true;
	}
};

JoltPhysicsDirectSpaceState3D::JoltPhysicsDirectSpaceState3D(JoltSpace3D* p_space)
	: space(p_space) { }

bool JoltPhysicsDirectSpaceState3D::_intersect_ray(
	const Vector3& p_from,
	const Vector3& p_to,
	int64_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	bool p_hit_from_inside,
	bool p_hit_back_faces,
	PhysicsServer3DExtensionRayResult* p_result
) {
	const JPH::Vec3 from = to_jolt(p_from);
	const JPH::Vec3 to = to_jolt(p_to);
	const JPH::Vec3 vector = to - from;

	const JPH::NarrowPhaseQuery& query = space->get_narrow_phase_query(false);

	const JPH::RRayCast ray(from, vector);

	JPH::RayCastSettings settings;
	settings.mTreatConvexAsSolid = p_hit_from_inside;
	settings.mBackFaceMode = p_hit_back_faces ? JPH::EBackFaceMode::CollideWithBackFaces
											  : JPH::EBackFaceMode::IgnoreBackFaces;

	JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;

	query.CastRay(
		ray,
		settings,
		collector,
		JoltQueryBroadPhaseLayerFilter3D(),
		JoltQueryObjectLayerFilter3D(),
		JoltQueryBodyFilter3D(
			(uint32_t)p_collision_mask,
			p_collide_with_bodies,
			p_collide_with_areas
		),
		JoltQueryShapeFilter3D()
	);

	if (!collector.HadHit()) {
		return false;
	}

	const JPH::BodyID& body_id = collector.mHit.mBodyID;
	const JPH::SubShapeID& subshape_id = collector.mHit.mSubShapeID2;

	const JoltBodyAccessRead3D body_access(*space, body_id, false);
	ERR_FAIL_COND_D(!body_access.is_valid());

	const JPH::Body& body = body_access.get_body();
	const JPH::Vec3 position = ray.GetPointOnRay(collector.mHit.mFraction);
	const JPH::Vec3 normal = body.GetWorldSpaceSurfaceNormal(subshape_id, position);

	auto* object = reinterpret_cast<JoltCollisionObject3D*>(body.GetUserData());
	const auto object_id = (uint64_t)object->get_instance_id();

	const JPH::Shape& shape = *body.GetShape();
	const auto shape_idx = (int)shape.GetSubShapeUserData(subshape_id);

	p_result->position = to_godot(position);
	p_result->normal = to_godot(normal);
	p_result->rid = object->get_rid();
	p_result->collider_id = object_id;
	p_result->collider = ObjectDB::get_instance(object_id);
	p_result->shape = shape_idx;

	return true;
}

int64_t JoltPhysicsDirectSpaceState3D::_intersect_point(
	[[maybe_unused]] const Vector3& p_position,
	[[maybe_unused]] int64_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] PhysicsServer3DExtensionShapeResult* p_results,
	[[maybe_unused]] int64_t p_max_results
) {
	ERR_FAIL_D_NOT_IMPL();
}

int64_t JoltPhysicsDirectSpaceState3D::_intersect_shape(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] int64_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] PhysicsServer3DExtensionShapeResult* p_result_count,
	[[maybe_unused]] int64_t p_max_results
) {
	ERR_FAIL_D_NOT_IMPL();
}

bool JoltPhysicsDirectSpaceState3D::_cast_motion(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] int64_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] float* p_closest_safe,
	[[maybe_unused]] float* p_closest_unsafe,
	[[maybe_unused]] PhysicsServer3DExtensionShapeRestInfo* p_info
) {
	ERR_FAIL_D_NOT_IMPL();
}

bool JoltPhysicsDirectSpaceState3D::_collide_shape(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] int64_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] void* p_results,
	[[maybe_unused]] int64_t p_max_results,
	[[maybe_unused]] int32_t* p_result_count
) {
	ERR_FAIL_D_NOT_IMPL();
}

bool JoltPhysicsDirectSpaceState3D::_rest_info(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] int64_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] PhysicsServer3DExtensionShapeRestInfo* p_rest_info
) {
	ERR_FAIL_D_NOT_IMPL();
}

Vector3 JoltPhysicsDirectSpaceState3D::_get_closest_point_to_object_volume(
	[[maybe_unused]] const RID& p_object,
	[[maybe_unused]] const Vector3& p_point
) const {
	ERR_FAIL_D_NOT_IMPL();
}
