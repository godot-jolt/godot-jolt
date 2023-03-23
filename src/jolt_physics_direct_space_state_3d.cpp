#include "jolt_physics_direct_space_state_3d.hpp"

#include "jolt_area_3d.hpp"
#include "jolt_body_3d.hpp"
#include "jolt_collision_object_3d.hpp"
#include "jolt_motion_filter_3d.hpp"
#include "jolt_physics_server_3d.hpp"
#include "jolt_query_collectors.hpp"
#include "jolt_query_filter_3d.hpp"
#include "jolt_shape_3d.hpp"
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

	JoltQueryCollectorClosest<JPH::CastRayCollector> collector;

	space->get_narrow_phase_query()
		.CastRay(ray, settings, collector, query_filter, query_filter, query_filter);

	if (!collector.had_hit()) {
		return false;
	}

	const JPH::RayCastResult& hit = collector.get_hit();

	const JPH::BodyID& body_id = hit.mBodyID;
	const JPH::SubShapeID& sub_shape_id = hit.mSubShapeID2;

	const JoltReadableBody3D body = space->read_body(body_id);
	const JoltCollisionObject3D* object = body.as_object();
	ERR_FAIL_NULL_D(object);

	const JPH::Vec3 position = ray.GetPointOnRay(hit.mFraction);
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
	const Vector3& p_position,
	uint32_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	PhysicsServer3DExtensionShapeResult* p_results,
	int32_t p_max_results
) {
	if (p_max_results == 0) {
		return 0;
	}

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorAnyMulti<JPH::CollidePointCollector, 32> collector(p_max_results);

	space->get_narrow_phase_query()
		.CollidePoint(to_jolt(p_position), collector, query_filter, query_filter, query_filter);

	const int32_t hit_count = collector.get_hit_count();

	for (int32_t i = 0; i < hit_count; ++i) {
		const JPH::CollidePointResult& hit = collector.get_hit(i);

		const JoltReadableBody3D body = space->read_body(hit.mBodyID);
		const JoltCollisionObject3D* object = body.as_object();
		ERR_FAIL_NULL_D(object);

		const int32_t shape_index = object->find_shape_index(hit.mSubShapeID2);
		ERR_FAIL_COND_D(shape_index == -1);

		PhysicsServer3DExtensionShapeResult& result = *p_results++;

		result.rid = object->get_rid();
		result.collider_id = object->get_instance_id();
		result.collider = object->get_instance_unsafe();
		result.shape = shape_index;
	}

	return hit_count;
}

int32_t JoltPhysicsDirectSpaceState3D::_intersect_shape(
	const RID& p_shape_rid,
	const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	double p_margin,
	uint32_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	PhysicsServer3DExtensionShapeResult* p_results,
	int32_t p_max_results
) {
	if (p_max_results == 0) {
		return 0;
	}

	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltShape3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build((float)p_margin);
	ERR_FAIL_NULL_D(jolt_shape);

	const Vector3 center_of_mass = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = p_transform.translated_local(center_of_mass);
	Vector3 scale(1.0f, 1.0f, 1.0f);
	try_strip_scale(transform_com, scale);

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorAnyMulti<JPH::CollideShapeCollector, 32> collector(p_max_results);

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(scale),
		to_jolt(transform_com),
		JPH::CollideShapeSettings(),
		to_jolt(transform_com.origin),
		collector,
		query_filter,
		query_filter,
		query_filter
	);

	const int32_t hit_count = collector.get_hit_count();

	for (int32_t i = 0; i < hit_count; ++i) {
		const JPH::CollideShapeResult& hit = collector.get_hit(i);

		const JoltReadableBody3D body = space->read_body(hit.mBodyID2);
		const JoltCollisionObject3D* object = body.as_object();
		ERR_FAIL_NULL_D(object);

		const int32_t shape_index = object->find_shape_index(hit.mSubShapeID2);
		ERR_FAIL_COND_D(shape_index == -1);

		PhysicsServer3DExtensionShapeResult& result = *p_results++;

		result.rid = object->get_rid();
		result.collider_id = object->get_instance_id();
		result.collider = object->get_instance_unsafe();
		result.shape = shape_index;
	}

	return hit_count;
}

bool JoltPhysicsDirectSpaceState3D::_cast_motion(
	const RID& p_shape_rid,
	const Transform3D& p_transform,
	const Vector3& p_motion,
	double p_margin,
	uint32_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	float* p_closest_safe,
	float* p_closest_unsafe,
	PhysicsServer3DExtensionShapeRestInfo* p_info
) {
	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltShape3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build((float)p_margin);
	ERR_FAIL_NULL_D(jolt_shape);

	const Vector3 center_of_mass = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = p_transform.translated_local(center_of_mass);
	Vector3 scale(1.0f, 1.0f, 1.0f);
	try_strip_scale(transform_com, scale);

	JPH::ShapeCastSettings settings;
	settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = true;

	const Vector3& base_offset = transform_com.origin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorClosest<JPH::CastShapeCollector> collector;

	space->get_narrow_phase_query().CastShape(
		JPH::RShapeCast(jolt_shape, to_jolt(scale), to_jolt(transform_com), to_jolt(p_motion)),
		settings,
		to_jolt(base_offset),
		collector,
		query_filter,
		query_filter,
		query_filter
	);

	if (!collector.had_hit()) {
		return false;
	}

	const JPH::ShapeCastResult& hit = collector.get_hit();

	if (p_info != nullptr) {
		const JoltReadableBody3D body = space->read_body(hit.mBodyID2);
		const JoltCollisionObject3D* object = body.as_object();
		ERR_FAIL_NULL_D(object);

		const int32_t shape_index = object->find_shape_index(hit.mSubShapeID2);
		ERR_FAIL_COND_D(shape_index == -1);

		const Vector3 hit_point = base_offset + to_godot(hit.mContactPointOn2);

		p_info->point = hit_point;
		p_info->normal = to_godot(-hit.mPenetrationAxis.Normalized());
		p_info->rid = object->get_rid();
		p_info->collider_id = object->get_instance_id();
		p_info->shape = shape_index;
		p_info->linear_velocity = object->get_velocity_at_position(hit_point, false);
	}

	const float small_number = 0.000001f;
	const float motion_length = p_motion.length();
	const float nudge_epsilon = max(small_number * motion_length, small_number);
	const float nudge_safe = settings.mCollisionTolerance + nudge_epsilon;
	const float nudge_unsafe = nudge_safe + shape->get_margin();
	const float nudge_fraction_safe = max(nudge_safe / motion_length, FLT_EPSILON);
	const float nudge_fraction_unsafe = max(nudge_unsafe / motion_length, FLT_EPSILON);

	*p_closest_safe = max(hit.mFraction - nudge_fraction_safe, 0.0f);
	*p_closest_unsafe = min(hit.mFraction + nudge_fraction_unsafe, 1.0f);

	return true;
}

bool JoltPhysicsDirectSpaceState3D::_collide_shape(
	const RID& p_shape_rid,
	const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	double p_margin,
	uint32_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	void* p_results,
	int32_t p_max_results,
	int32_t* p_result_count
) {
	if (p_max_results == 0) {
		return false;
	}

	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltShape3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build((float)p_margin);
	ERR_FAIL_NULL_D(jolt_shape);

	const Vector3 center_of_mass = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = p_transform.translated_local(center_of_mass);
	Vector3 scale(1.0f, 1.0f, 1.0f);
	try_strip_scale(transform_com, scale);

	const Vector3& base_offset = transform_com.origin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorAnyMulti<JPH::CollideShapeCollector, 32> collector(p_max_results);

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(scale),
		to_jolt(transform_com),
		JPH::CollideShapeSettings(),
		to_jolt(base_offset),
		collector,
		query_filter,
		query_filter,
		query_filter
	);

	auto* results = static_cast<Vector3*>(p_results);

	const int32_t hit_count = collector.get_hit_count();

	for (int32_t i = 0; i < hit_count; ++i) {
		const JPH::CollideShapeResult& hit = collector.get_hit(i);

		*results++ = base_offset + to_godot(hit.mContactPointOn1);
		*results++ = base_offset + to_godot(hit.mContactPointOn2);
	}

	*p_result_count = hit_count;

	return true;
}

bool JoltPhysicsDirectSpaceState3D::_rest_info(
	const RID& p_shape_rid,
	const Transform3D& p_transform,
	[[maybe_unused]] const Vector3& p_motion,
	double p_margin,
	uint32_t p_collision_mask,
	bool p_collide_with_bodies,
	bool p_collide_with_areas,
	PhysicsServer3DExtensionShapeRestInfo* p_info
) {
	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltShape3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build((float)p_margin);
	ERR_FAIL_NULL_D(jolt_shape);

	const Vector3 center_of_mass = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = p_transform.translated_local(center_of_mass);
	Vector3 scale(1.0f, 1.0f, 1.0f);
	try_strip_scale(transform_com, scale);

	const Vector3& base_offset = transform_com.origin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorClosest<JPH::CollideShapeCollector> collector;

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(scale),
		to_jolt(transform_com),
		JPH::CollideShapeSettings(),
		to_jolt(base_offset),
		collector,
		query_filter,
		query_filter,
		query_filter
	);

	if (!collector.had_hit()) {
		return false;
	}

	const JPH::CollideShapeResult& hit = collector.get_hit();

	const JoltReadableBody3D body = space->read_body(hit.mBodyID2);
	const JoltCollisionObject3D* object = body.as_object();
	ERR_FAIL_NULL_D(object);

	const int32_t shape_index = object->find_shape_index(hit.mSubShapeID2);
	ERR_FAIL_COND_D(shape_index == -1);

	const Vector3 hit_point = base_offset + to_godot(hit.mContactPointOn2);

	p_info->point = hit_point;
	p_info->normal = to_godot(-hit.mPenetrationAxis.Normalized());
	p_info->rid = object->get_rid();
	p_info->collider_id = object->get_instance_id();
	p_info->shape = shape_index;
	p_info->linear_velocity = object->get_velocity_at_position(hit_point, false);

	return true;
}

Vector3 JoltPhysicsDirectSpaceState3D::_get_closest_point_to_object_volume(
	const RID& p_object,
	const Vector3& p_point
) const {
	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltCollisionObject3D* object = physics_server->get_area(p_object);

	if (object == nullptr) {
		object = physics_server->get_body(p_object);
	}

	ERR_FAIL_NULL_D(object);
	ERR_FAIL_COND_D(object->get_space() != space);

	const JoltReadableBody3D body = space->read_body(*object);
	const JPH::TransformedShape shape = body->GetTransformedShape();

	JoltQueryCollectorAll<JPH::TransformedShapeCollector, 32> collector;
	shape.CollectTransformedShapes(body->GetWorldSpaceBounds(), collector);

	const JPH::Vec3 point = to_jolt(p_point);

	float closest_distance_sq = FLT_MAX;
	JPH::Vec3 closest_point = JPH::Vec3::sZero();

	bool found_point = false;

	for (int32_t i = 0; i < collector.get_hit_count(); ++i) {
		const JPH::TransformedShape& sub_shape_transformed = collector.get_hit(i);
		const JPH::Shape& sub_shape = *sub_shape_transformed.mShape;

		if (sub_shape.GetType() != JPH::EShapeType::Convex) {
			continue;
		}

		const auto& sub_shape_convex = static_cast<const JPH::ConvexShape&>(sub_shape);

		JPH::GJKClosestPoint gjk;

		// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
		JPH::ConvexShape::SupportBuffer shape_support_buffer;

		const JPH::ConvexShape::Support* shape_support = sub_shape_convex.GetSupportFunction(
			JPH::ConvexShape::ESupportMode::IncludeConvexRadius,
			shape_support_buffer,
			sub_shape_transformed.GetShapeScale()
		);

		const JPH::Quat& sub_shape_rotation = sub_shape_transformed.mShapeRotation;
		const JPH::Vec3& sub_shape_pos_com = sub_shape_transformed.mShapePositionCOM;
		const JPH::Mat44 sub_shape_3x3 = JPH::RMat44::sRotation(sub_shape_rotation);
		const JPH::Vec3 sub_shape_com_local = sub_shape.GetCenterOfMass();
		const JPH::Vec3 sub_shape_com = sub_shape_3x3.Multiply3x3(sub_shape_com_local);
		const JPH::Vec3 sub_shape_pos = sub_shape_pos_com - sub_shape_com;
		const JPH::Mat44 sub_shape_4x4 = sub_shape_3x3.PostTranslated(sub_shape_pos);
		const JPH::Mat44 sub_shape_4x4_inv = sub_shape_4x4.InversedRotationTranslation();

		JPH::PointConvexSupport point_support = {};
		point_support.mPoint = sub_shape_4x4_inv * point;

		JPH::Vec3 separating_axis = JPH::Vec3::sAxisX();
		JPH::Vec3 point_on_a = JPH::Vec3::sZero();
		JPH::Vec3 point_on_b = JPH::Vec3::sZero();

		const float distance_sq = gjk.GetClosestPoints(
			*shape_support,
			point_support,
			JPH::cDefaultCollisionTolerance,
			FLT_MAX,
			separating_axis,
			point_on_a,
			point_on_b
		);

		if (distance_sq == 0.0f) {
			closest_point = point;
			found_point = true;
			break;
		}

		if (distance_sq < closest_distance_sq) {
			closest_distance_sq = distance_sq;
			closest_point = sub_shape_4x4 * point_on_a;
			found_point = true;
		}
	}

	if (found_point) {
		return to_godot(closest_point);
	} else {
		return to_godot(body->GetPosition());
	}
}

bool JoltPhysicsDirectSpaceState3D::test_body_motion(
	const JoltBody3D& p_body,
	const Transform3D& p_transform,
	const Vector3& p_motion,
	float p_margin,
	int32_t p_max_collisions,
	bool p_collide_separation_ray,
	PhysicsServer3DExtensionMotionResult* p_result
) const {
	const JPH::Shape* jolt_shape = p_body.get_jolt_shape();

	const Vector3 center_of_mass = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = p_transform.translated_local(center_of_mass);
	Vector3 scale(1.0f, 1.0f, 1.0f);
	try_strip_scale(transform_com, scale);

	const Vector3 motion_direction = p_motion.normalized();

	Vector3 recover_motion;

	const bool recovered = body_motion_recover(
		p_body,
		transform_com,
		scale,
		motion_direction,
		p_margin,
		recover_motion
	);

	float safe_fraction = 1.0f;
	float unsafe_fraction = 1.0f;

	const bool hit = body_motion_cast(
		p_body,
		transform_com,
		scale,
		p_motion,
		p_collide_separation_ray,
		safe_fraction,
		unsafe_fraction
	);

	bool collided = false;

	if (hit || (recovered /* && p_recovery_as_collision */)) {
		collided = body_motion_collide(
			p_body,
			transform_com.translated(p_motion * unsafe_fraction),
			scale,
			motion_direction,
			p_margin,
			min(p_max_collisions, 32),
			p_result->collisions,
			p_result->collision_count
		);
	}

	if (collided) {
		const PhysicsServer3DExtensionMotionCollision& deepest = p_result->collisions[0];

		p_result->travel = recover_motion + p_motion * safe_fraction;
		p_result->remainder = p_motion - p_motion * safe_fraction;
		p_result->collision_depth = deepest.depth;
		p_result->collision_safe_fraction = safe_fraction;
		p_result->collision_unsafe_fraction = unsafe_fraction;
	} else {
		p_result->travel = recover_motion + p_motion;
		p_result->remainder = Vector3();
		p_result->collision_depth = 0.0f;
		p_result->collision_safe_fraction = 1.0f;
		p_result->collision_unsafe_fraction = 1.0f;
		p_result->collision_count = 0;
	}

	return collided;
}

bool JoltPhysicsDirectSpaceState3D::body_motion_recover(
	const JoltBody3D& p_body,
	Transform3D& p_transform_com,
	const Vector3& p_scale,
	const Vector3& p_direction,
	float p_margin,
	Vector3& p_recover_motion
) const {
	const JPH::Shape* jolt_shape = p_body.get_jolt_shape();

	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = to_jolt(p_direction);
	settings.mMaxSeparationDistance = p_margin;

	const JoltMotionFilter3D motion_filter(p_body);

	bool recovered = false;

	for (int32_t i = 0; i < 4; ++i) {
		JoltQueryCollectorClosest<JPH::CollideShapeCollector> collector;

		space->get_narrow_phase_query().CollideShape(
			jolt_shape,
			to_jolt(p_scale),
			to_jolt(p_transform_com),
			settings,
			to_jolt(p_transform_com.origin),
			collector,
			motion_filter,
			motion_filter,
			motion_filter,
			motion_filter
		);

		if (!collector.had_hit()) {
			break;
		}

		const JPH::CollideShapeResult& hit = collector.get_hit();

		const float depth = hit.mPenetrationDepth + p_margin;

		if (depth <= 0.0f) {
			break;
		}

		const Vector3 contact_normal = to_godot(-hit.mPenetrationAxis.Normalized());
		const Vector3 recover_motion = contact_normal * depth;

		p_recover_motion += recover_motion;
		p_transform_com.origin += recover_motion;

		recovered = true;
	}

	return recovered;
}

bool JoltPhysicsDirectSpaceState3D::body_motion_cast(
	const JoltBody3D& p_body,
	const Transform3D& p_transform_com,
	const Vector3& p_scale,
	const Vector3& p_motion,
	bool p_collide_separation_ray,
	float& p_safe_fraction,
	float& p_unsafe_fraction
) const {
	const JPH::Shape* jolt_shape = p_body.get_jolt_shape();

	const float motion_length = p_motion.length();
	const Vector3 direction = motion_length != 0.0f ? p_motion / motion_length : p_motion;

	JPH::ShapeCastSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = to_jolt(direction);
	settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = true;

	const JoltMotionFilter3D motion_filter(p_body, p_collide_separation_ray);

	JoltQueryCollectorClosest<JPH::CastShapeCollector> collector;

	space->get_narrow_phase_query().CastShape(
		JPH::RShapeCast(jolt_shape, to_jolt(p_scale), to_jolt(p_transform_com), to_jolt(p_motion)),
		settings,
		to_jolt(p_transform_com.origin),
		collector,
		motion_filter,
		motion_filter,
		motion_filter,
		motion_filter
	);

	if (!collector.had_hit()) {
		p_safe_fraction = 1.0f;
		p_unsafe_fraction = 1.0f;

		return false;
	}

	const JPH::ShapeCastResult& hit = collector.get_hit();

	const JoltShape3D* shape = p_body.find_shape(hit.mSubShapeID1);
	ERR_FAIL_NULL_D(shape);

	const float small_number = 0.000001f;
	const float nudge_epsilon = max(small_number * motion_length, small_number);
	const float nudge_safe = settings.mCollisionTolerance + nudge_epsilon;
	const float nudge_unsafe = nudge_safe + shape->get_margin();
	const float nudge_fraction_safe = max(nudge_safe / motion_length, FLT_EPSILON);
	const float nudge_fraction_unsafe = max(nudge_unsafe / motion_length, FLT_EPSILON);

	p_safe_fraction = max(hit.mFraction - nudge_fraction_safe, 0.0f);
	p_unsafe_fraction = min(hit.mFraction + nudge_fraction_unsafe, 1.0f);

	return true;
}

bool JoltPhysicsDirectSpaceState3D::body_motion_collide(
	const JoltBody3D& p_body,
	const Transform3D& p_transform_com,
	const Vector3& p_scale,
	const Vector3& p_direction,
	float p_margin,
	int32_t p_max_collisions,
	PhysicsServer3DExtensionMotionCollision* p_collisions,
	int32_t& p_collision_count
) const {
	const JPH::Shape* jolt_shape = p_body.get_jolt_shape();

	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = to_jolt(p_direction);
	settings.mMaxSeparationDistance = p_margin;

	const Vector3& base_offset = p_transform_com.origin;

	const JoltMotionFilter3D motion_filter(p_body);

	JoltQueryCollectorClosestMulti<JPH::CollideShapeCollector, 32> collector(p_max_collisions);

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(p_scale),
		to_jolt(p_transform_com),
		settings,
		to_jolt(base_offset),
		collector,
		motion_filter,
		motion_filter,
		motion_filter,
		motion_filter
	);

	if (!collector.had_hit()) {
		p_collision_count = 0;

		return false;
	}

	p_collision_count = collector.get_hit_count();

	for (int32_t i = 0; i < p_collision_count; ++i) {
		const JPH::CollideShapeResult& hit = collector.get_hit(i);

		const JoltReadableBody3D collider_jolt_body = space->read_body(hit.mBodyID2);
		const JoltCollisionObject3D* collider = collider_jolt_body.as_object();
		ERR_FAIL_NULL_D(collider);

		const Vector3 position = base_offset + to_godot(hit.mContactPointOn2);

		const int32_t local_shape = p_body.find_shape_index(hit.mSubShapeID1);
		ERR_FAIL_COND_D(local_shape == -1);

		const int32_t collider_shape = collider->find_shape_index(hit.mSubShapeID2);
		ERR_FAIL_COND_D(collider_shape == -1);

		PhysicsServer3DExtensionMotionCollision& collision = *p_collisions++;

		collision.position = position;
		collision.normal = to_godot(-hit.mPenetrationAxis.Normalized());
		collision.collider_velocity = collider->get_velocity_at_position(position, false);
		collision.collider_angular_velocity = collider->get_angular_velocity(false);
		collision.depth = hit.mPenetrationDepth + p_margin;
		collision.local_shape = local_shape;
		collision.collider_id = collider->get_instance_id();
		collision.collider = collider->get_rid();
		collision.collider_shape = collider_shape;
	}

	return true;
}
