#include "jolt_physics_direct_space_state_3d.hpp"

#include "jolt_collision_object_3d.hpp"
#include "jolt_query_filter_3d.hpp"
#include "jolt_space_3d.hpp"

JoltPhysicsDirectSpaceState3D::JoltPhysicsDirectSpaceState3D(JoltSpace3D* p_space)
	: space(p_space) { }

bool JoltPhysicsDirectSpaceState3D::_intersect_ray(
	const Vector3& p_from,
	const Vector3& p_to,
	uint32_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	bool p_hit_from_inside,
	bool p_hit_back_faces,
	PhysicsServer3DExtensionRayResult* p_result
) {
	const JPH::NarrowPhaseQuery& query = space->get_narrow_phase_query();

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	const JPH::Vec3 from = to_jolt(p_from);
	const JPH::Vec3 to = to_jolt(p_to);
	const JPH::Vec3 vector = to - from;
	const JPH::RRayCast ray(from, vector);

	JPH::RayCastSettings settings;
	settings.mTreatConvexAsSolid = p_hit_from_inside;
	settings.mBackFaceMode = p_hit_back_faces ? JPH::EBackFaceMode::CollideWithBackFaces
											  : JPH::EBackFaceMode::IgnoreBackFaces;

	JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;

	query.CastRay(ray, settings, collector, query_filter, query_filter, query_filter);

	if (!collector.HadHit()) {
		return false;
	}

	const JPH::BodyID& body_id = collector.mHit.mBodyID;
	const JPH::SubShapeID& sub_shape_id = collector.mHit.mSubShapeID2;

	const JoltReadableBody3D body = space->read_body(body_id);
	const JoltCollisionObject3D* object = body.as_object();
	ERR_FAIL_NULL_D(object);

	const JPH::Vec3 position = ray.GetPointOnRay(collector.mHit.mFraction);
	const JPH::Vec3 normal = body->GetWorldSpaceSurfaceNormal(sub_shape_id, position);

	const ObjectID object_id = object->get_instance_id();

	const int32_t shape_index = object->find_shape_index(sub_shape_id);
	ERR_FAIL_COND_D(shape_index == -1);

	p_result->position = to_godot(position);
	p_result->normal = to_godot(normal);
	p_result->rid = object->get_rid();
	p_result->collider_id = object_id;
	p_result->collider = object->get_instance_unsafe();
	p_result->shape = shape_index;

	return true;
}

int32_t JoltPhysicsDirectSpaceState3D::_intersect_point(
	[[maybe_unused]] const Vector3& p_position,
	[[maybe_unused]] uint32_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] PhysicsServer3DExtensionShapeResult* p_results,
	[[maybe_unused]] int32_t p_max_results
) {
	ERR_FAIL_D_NOT_IMPL();
}

int32_t JoltPhysicsDirectSpaceState3D::_intersect_shape(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] uint32_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] PhysicsServer3DExtensionShapeResult* p_result_count,
	[[maybe_unused]] int32_t p_max_results
) {
	ERR_FAIL_D_NOT_IMPL();
}

bool JoltPhysicsDirectSpaceState3D::_cast_motion(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] uint32_t p_collision_mask,
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
	[[maybe_unused]] uint32_t p_collision_mask,
	[[maybe_unused]] bool p_collide_with_bodies,
	[[maybe_unused]] bool p_collide_with_areas,
	[[maybe_unused]] void* p_results,
	[[maybe_unused]] int32_t p_max_results,
	[[maybe_unused]] int32_t* p_result_count
) {
	ERR_FAIL_D_NOT_IMPL();
}

bool JoltPhysicsDirectSpaceState3D::_rest_info(
	[[maybe_unused]] const RID& p_shape_rid,
	[[maybe_unused]] const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	[[maybe_unused]] double p_margin,
	[[maybe_unused]] uint32_t p_collision_mask,
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
