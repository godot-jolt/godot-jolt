#include "jolt_physics_direct_space_state_3d.hpp"

#include "objects/jolt_area_impl_3d.hpp"
#include "objects/jolt_body_impl_3d.hpp"
#include "objects/jolt_object_impl_3d.hpp"
#include "servers/jolt_physics_server_3d.hpp"
#include "servers/jolt_project_settings.hpp"
#include "shapes/jolt_custom_motion_shape.hpp"
#include "shapes/jolt_shape_impl_3d.hpp"
#include "spaces/jolt_motion_filter_3d.hpp"
#include "spaces/jolt_query_collectors.hpp"
#include "spaces/jolt_query_filter_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

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
	bool p_pick_ray,
	PhysicsServer3DExtensionRayResult* p_result
) {
	const JoltQueryFilter3D query_filter(
		*this,
		p_collision_mask,
		p_collide_with_bodies,
		p_collide_with_areas,
		p_pick_ray
	);

	const JPH::Vec3 from = to_jolt(p_from);
	const JPH::Vec3 to = to_jolt(p_to);
	const JPH::Vec3 vector = to - from;
	const JPH::RRayCast ray(from, vector);

	JPH::RayCastSettings settings;
	settings.mTreatConvexAsSolid = p_hit_from_inside;
	settings.mBackFaceMode = p_hit_back_faces
		? JPH::EBackFaceMode::CollideWithBackFaces
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
	const JoltObjectImpl3D* object = body.as_object();
	ERR_FAIL_NULL_D(object);

	const JPH::Vec3 position = ray.GetPointOnRay(hit.mFraction);

	JPH::Vec3 normal = JPH::Vec3::sZero();

	if (!p_hit_from_inside || hit.mFraction > 0.0f) {
		normal = body->GetWorldSpaceSurfaceNormal(sub_shape_id, position);

		// HACK(mihe): If we got a back-face normal we need to flip it
		if (normal.Dot(vector) > 0) {
			normal = -normal;
		}
	}

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
		const JoltObjectImpl3D* object = body.as_object();
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

	JoltShapeImpl3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build();
	ERR_FAIL_NULL_D(jolt_shape);

	Vector3 scale;
	const Transform3D transform = Math::decomposed(p_transform, scale);
	const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
	const Transform3D transform_com = transform.translated_local(com_scaled);

	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = (float)p_margin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorAnyMulti<JPH::CollideShapeCollector, 32> collector(p_max_results);

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(scale),
		to_jolt(transform_com),
		settings,
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
		const JoltObjectImpl3D* object = body.as_object();
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
	// HACK(mihe): This rest info parameter doesn't seem to be used anywhere within Godot, and isn't
	// exposed in the bindings, so this will be unsupported until anyone actually needs it.
	ERR_FAIL_COND_D_MSG(
		p_info != nullptr,
		"Providing rest info as part of a shape-cast is not supported by Godot Jolt."
	);

	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltShapeImpl3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build();
	ERR_FAIL_NULL_D(jolt_shape);

	Vector3 scale;
	const Transform3D transform = Math::decomposed(p_transform, scale);
	const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = transform.translated_local(com_scaled);

	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = (float)p_margin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	_cast_motion_impl(
		*jolt_shape,
		transform_com,
		scale,
		p_motion,
		true,
		settings,
		query_filter,
		query_filter,
		query_filter,
		JPH::ShapeFilter(),
		*p_closest_safe,
		*p_closest_unsafe
	);

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
	*p_result_count = 0;

	if (p_max_results == 0) {
		return false;
	}

	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltShapeImpl3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build();
	ERR_FAIL_NULL_D(jolt_shape);

	Vector3 scale;
	const Transform3D transform = Math::decomposed(p_transform, scale);
	const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
	const Transform3D transform_com = transform.translated_local(com_scaled);

	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = (float)p_margin;

	const Vector3& base_offset = transform_com.origin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorAnyMulti<JPH::CollideShapeCollector, 32> collector(p_max_results);

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(scale),
		to_jolt(transform_com),
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

	auto* results = static_cast<Vector3*>(p_results);

	*p_result_count = collector.get_hit_count();

	for (int32_t i = 0; i < *p_result_count; ++i) {
		const JPH::CollideShapeResult& hit = collector.get_hit(i);

		const Vector3 penetration_axis = to_godot(hit.mPenetrationAxis.Normalized());
		const Vector3 margin_offset = penetration_axis * (float)p_margin;

		*results++ = base_offset + to_godot(hit.mContactPointOn1) + margin_offset;
		*results++ = base_offset + to_godot(hit.mContactPointOn2);
	}

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

	JoltShapeImpl3D* shape = physics_server->get_shape(p_shape_rid);
	ERR_FAIL_NULL_D(shape);

	const JPH::ShapeRefC jolt_shape = shape->try_build();
	ERR_FAIL_NULL_D(jolt_shape);

	Vector3 scale;
	const Transform3D transform = Math::decomposed(p_transform, scale);
	const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
	const Transform3D transform_com = transform.translated_local(com_scaled);

	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = (float)p_margin;

	const Vector3& base_offset = transform_com.origin;

	const JoltQueryFilter3D
		query_filter(*this, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	JoltQueryCollectorClosest<JPH::CollideShapeCollector> collector;

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		to_jolt(scale),
		to_jolt(transform_com),
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

	const JPH::CollideShapeResult& hit = collector.get_hit();

	const JoltReadableBody3D body = space->read_body(hit.mBodyID2);
	const JoltObjectImpl3D* object = body.as_object();
	ERR_FAIL_NULL_D(object);

	const int32_t shape_index = object->find_shape_index(hit.mSubShapeID2);
	ERR_FAIL_COND_D(shape_index == -1);

	const Vector3 hit_point = base_offset + to_godot(hit.mContactPointOn2);

	p_info->point = hit_point;
	p_info->normal = to_godot(-hit.mPenetrationAxis.Normalized());
	p_info->rid = object->get_rid();
	p_info->collider_id = object->get_instance_id();
	p_info->shape = shape_index;
	p_info->linear_velocity = object->get_velocity_at_position(hit_point);

	return true;
}

Vector3 JoltPhysicsDirectSpaceState3D::_get_closest_point_to_object_volume(
	const RID& p_object,
	const Vector3& p_point
) const {
	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());

	JoltObjectImpl3D* object = physics_server->get_area(p_object);

	if (object == nullptr) {
		object = physics_server->get_body(p_object);
	}

	ERR_FAIL_NULL_D(object);
	ERR_FAIL_COND_D(object->get_space() != space);

	const JoltReadableBody3D body = space->read_body(*object);
	const JPH::TransformedShape root_shape = body->GetTransformedShape();

	JoltQueryCollectorAll<JPH::TransformedShapeCollector, 32> collector;
	root_shape.CollectTransformedShapes(body->GetWorldSpaceBounds(), collector);

	const JPH::Vec3 point = to_jolt(p_point);

	float closest_distance_sq = FLT_MAX;
	JPH::Vec3 closest_point = JPH::Vec3::sZero();

	bool found_point = false;

	for (int32_t i = 0; i < collector.get_hit_count(); ++i) {
		const JPH::TransformedShape& shape_transformed = collector.get_hit(i);
		const JPH::Shape& shape = *shape_transformed.mShape;

		if (shape.GetType() != JPH::EShapeType::Convex) {
			continue;
		}

		const auto& shape_convex = static_cast<const JPH::ConvexShape&>(shape);

		JPH::GJKClosestPoint gjk;

		// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
		JPH::ConvexShape::SupportBuffer shape_support_buffer;

		const JPH::ConvexShape::Support* shape_support = shape_convex.GetSupportFunction(
			JPH::ConvexShape::ESupportMode::IncludeConvexRadius,
			shape_support_buffer,
			shape_transformed.GetShapeScale()
		);

		const JPH::Quat& shape_rotation = shape_transformed.mShapeRotation;
		const JPH::Vec3& shape_pos_com = shape_transformed.mShapePositionCOM;
		const JPH::Mat44 shape_3x3 = JPH::RMat44::sRotation(shape_rotation);
		const JPH::Vec3 shape_com_local = shape.GetCenterOfMass();
		const JPH::Vec3 shape_com = shape_3x3.Multiply3x3(shape_com_local);
		const JPH::Vec3 shape_pos = shape_pos_com - shape_com;
		const JPH::Mat44 shape_4x4 = shape_3x3.PostTranslated(shape_pos);
		const JPH::Mat44 shape_4x4_inv = shape_4x4.InversedRotationTranslation();

		JPH::PointConvexSupport point_support = {};
		point_support.mPoint = shape_4x4_inv * point;

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
			closest_point = shape_4x4 * point_on_a;
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
	const JoltBodyImpl3D& p_body,
	const Transform3D& p_transform,
	const Vector3& p_motion,
	float p_margin,
	int32_t p_max_collisions,
	bool p_collide_separation_ray,
	bool p_recovery_as_collision,
	PhysicsServer3DExtensionMotionResult* p_result
) const {
	p_margin = max(p_margin, 0.0001f);
	p_max_collisions = min(p_max_collisions, 32);

	Vector3 scale;
	Transform3D transform = Math::decomposed(p_transform, scale);

	Vector3 recovery;
	const bool recovered = _body_motion_recover(p_body, transform, p_margin, recovery);

	transform.origin += recovery;

	float safe_fraction = 1.0f;
	float unsafe_fraction = 1.0f;

	const bool hit = _body_motion_cast(
		p_body,
		transform,
		scale,
		p_motion,
		p_collide_separation_ray,
		safe_fraction,
		unsafe_fraction
	);

	bool collided = false;

	if (hit || (recovered && p_recovery_as_collision)) {
		collided = _body_motion_collide(
			p_body,
			transform.translated(p_motion * unsafe_fraction),
			p_motion.length(),
			p_margin,
			p_max_collisions,
			p_result
		);
	}

	if (p_result == nullptr) {
		return collided;
	}

	if (collided) {
		const PhysicsServer3DExtensionMotionCollision& deepest = p_result->collisions[0];

		p_result->travel = recovery + p_motion * safe_fraction;
		p_result->remainder = p_motion - p_motion * safe_fraction;
		p_result->collision_depth = deepest.depth;
		p_result->collision_safe_fraction = safe_fraction;
		p_result->collision_unsafe_fraction = unsafe_fraction;
	} else {
		p_result->travel = recovery + p_motion;
		p_result->remainder = Vector3();
		p_result->collision_depth = 0.0f;
		p_result->collision_safe_fraction = 1.0f;
		p_result->collision_unsafe_fraction = 1.0f;
		p_result->collision_count = 0;
	}

	return collided;
}

bool JoltPhysicsDirectSpaceState3D::_cast_motion_impl(
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
	float& p_closest_safe,
	float& p_closest_unsafe
) const {
	p_closest_safe = 1.0f;
	p_closest_unsafe = 1.0f;

	ERR_FAIL_COND_D_MSG(
		p_jolt_shape.GetType() != JPH::EShapeType::Convex,
		"Shape-casting with non-convex shapes is not supported by Godot Jolt."
	);

	const float motion_length = p_motion.length();

	if (p_ignore_overlaps && motion_length == 0.0f) {
		return false;
	}

	const JPH::Mat44 transform_com = to_jolt(p_transform_com);
	const JPH::Vec3 scale = to_jolt(p_scale);
	const JPH::Vec3 motion = to_jolt(p_motion);
	const JPH::Vec3 motion_local = transform_com.Multiply3x3Transposed(motion);

	JPH::AABox aabb = p_jolt_shape.GetWorldSpaceBounds(transform_com, scale);
	JPH::AABox aabb_translated = aabb;
	aabb_translated.Translate(motion);
	aabb.Encapsulate(aabb_translated);

	JoltQueryCollectorAnyMulti<JPH::CollideShapeBodyCollector, 2048> aabb_collector;

	space->get_broad_phase_query()
		.CollideAABox(aabb, aabb_collector, p_broad_phase_layer_filter, p_object_layer_filter);

	if (!aabb_collector.had_hit()) {
		return false;
	}

	JoltCustomMotionShape motion_shape(static_cast<const JPH::ConvexShape&>(p_jolt_shape));

	auto collides = [&](const JPH::Body& p_other_body, float p_fraction) {
		motion_shape.set_motion(motion_local * p_fraction);

		const JPH::TransformedShape other_shape = p_other_body.GetTransformedShape();

		JoltQueryCollectorAny<JPH::CollideShapeCollector> collide_collector;

		JPH::CollisionDispatch::sCollideShapeVsShape(
			&motion_shape,
			other_shape.mShape,
			scale,
			other_shape.GetShapeScale(),
			transform_com,
			other_shape.GetCenterOfMassTransform(),
			JPH::SubShapeIDCreator(),
			JPH::SubShapeIDCreator(),
			p_settings,
			collide_collector,
			p_shape_filter
		);

		return collide_collector.had_hit();
	};

	// Figure out the number of steps we need in our binary search in order to achieve millimeter
	// precision, within reason. Derived from `2^-step_count * motion_length = 0.001`.
	const int32_t step_count = clamp(int32_t(logf(1000.0f * motion_length) / Math_LN2), 4, 16);

	bool collided = false;

	for (int32_t i = 0; i < aabb_collector.get_hit_count(); ++i) {
		const JPH::BodyID other_jolt_id = aabb_collector.get_hit(i);

		if (!p_body_filter.ShouldCollide(other_jolt_id)) {
			continue;
		}

		const JoltReadableBody3D other_jolt_body = space->read_body(other_jolt_id);

		if (!p_body_filter.ShouldCollideLocked(*other_jolt_body)) {
			continue;
		}

		if (!collides(*other_jolt_body, 1.0f)) {
			continue;
		}

		if (p_ignore_overlaps && collides(*other_jolt_body, 0.0f)) {
			continue;
		}

		float lo = 0.0f;
		float hi = 1.0f;
		float coeff = 0.5f;

		for (int j = 0; j < step_count; ++j) {
			const float fraction = lo + (hi - lo) * coeff;

			if (collides(*other_jolt_body, fraction)) {
				collided = true;

				hi = fraction;

				if (j == 0 || lo > 0.0f) {
					coeff = 0.5f;
				} else {
					coeff = 0.25f;
				}
			} else {
				lo = fraction;

				if (j == 0 || hi < 1.0f) {
					coeff = 0.5f;
				} else {
					coeff = 0.75f;
				}
			}
		}

		if (lo < p_closest_safe) {
			p_closest_safe = lo;
			p_closest_unsafe = hi;
		}
	}

	return collided;
}

bool JoltPhysicsDirectSpaceState3D::_body_motion_recover(
	const JoltBodyImpl3D& p_body,
	const Transform3D& p_transform,
	float p_margin,
	Vector3& p_recovery
) const {
	const int32_t recovery_iterations = JoltProjectSettings::get_kinematic_recovery_iterations();
	const float recovery_amount = JoltProjectSettings::get_kinematic_recovery_amount();

	const JPH::Shape* jolt_shape = p_body.get_jolt_shape();

	const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
	Transform3D transform_com = p_transform.translated_local(com_scaled);

	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mMaxSeparationDistance = p_margin;

	const Vector3& base_offset = transform_com.origin;

	const JoltMotionFilter3D motion_filter(p_body);

	JoltQueryCollectorAnyMulti<JPH::CollideShapeCollector, 32> collector;

	bool recovered = false;

	for (int32_t i = 0; i < recovery_iterations; ++i) {
		collector.reset();

		space->get_narrow_phase_query().CollideShape(
			jolt_shape,
			JPH::Vec3::sReplicate(1.0f),
			to_jolt(transform_com),
			settings,
			to_jolt(base_offset),
			collector,
			motion_filter,
			motion_filter,
			motion_filter,
			motion_filter
		);

		if (!collector.had_hit()) {
			break;
		}

		const int32_t hit_count = collector.get_hit_count();

		float combined_priority = 0.0;

		for (int j = 0; j < hit_count; j++) {
			const JPH::CollideShapeResult& hit = collector.get_hit(j);

			const JoltReadableBody3D other_jolt_body = space->read_body(hit.mBodyID2);
			const JoltBodyImpl3D* other_body = other_jolt_body.as_body();
			ERR_CONTINUE(other_body == nullptr);

			combined_priority += other_body->get_collision_priority();
		}

		const float average_priority = max(combined_priority / (float)hit_count, CMP_EPSILON);

		recovered = true;

		Vector3 recovery;

		for (int32_t j = 0; j < hit_count; ++j) {
			const JPH::CollideShapeResult& hit = collector.get_hit(j);

			const Vector3 penetration_axis = to_godot(hit.mPenetrationAxis.Normalized());
			const Vector3 margin_offset = penetration_axis * p_margin;

			const Vector3 point_on_1 = base_offset + to_godot(hit.mContactPointOn1) + margin_offset;
			const Vector3 point_on_2 = base_offset + to_godot(hit.mContactPointOn2);

			const float distance_to_1 = penetration_axis.dot(point_on_1 + recovery);
			const float distance_to_2 = penetration_axis.dot(point_on_2);

			const float penetration_depth = distance_to_1 - distance_to_2;

			if (penetration_depth <= 0.0f) {
				continue;
			}

			const JoltReadableBody3D other_jolt_body = space->read_body(hit.mBodyID2);
			const JoltBodyImpl3D* other_body = other_jolt_body.as_body();
			ERR_CONTINUE(other_body == nullptr);

			const float recovery_distance = penetration_depth * recovery_amount;
			const float other_priority = other_body->get_collision_priority();
			const float other_priority_normalized = other_priority / average_priority;
			const float scaled_recovery_distance = recovery_distance * other_priority_normalized;

			recovery -= penetration_axis * scaled_recovery_distance;
		}

		if (recovery == Vector3()) {
			break;
		}

		p_recovery += recovery;
		transform_com.origin += recovery;
	}

	return recovered;
}

bool JoltPhysicsDirectSpaceState3D::_body_motion_cast(
	const JoltBodyImpl3D& p_body,
	const Transform3D& p_transform,
	const Vector3& p_scale,
	const Vector3& p_motion,
	bool p_collide_separation_ray,
	float& p_safe_fraction,
	float& p_unsafe_fraction
) const {
	const Transform3D body_transform = p_transform.scaled_local(p_scale);

	const JPH::CollideShapeSettings settings;

	const JoltMotionFilter3D motion_filter(p_body, p_collide_separation_ray);

	bool collided = false;

	for (int32_t i = 0; i < p_body.get_shape_count(); ++i) {
		if (p_body.is_shape_disabled(i)) {
			continue;
		}

		JoltShapeImpl3D* shape = p_body.get_shape(i);

		if (!shape->is_convex()) {
			continue;
		}

		const JPH::ShapeRefC jolt_shape = shape->try_build();
		ERR_FAIL_NULL_D(jolt_shape);

		Vector3 scale;

		const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
		const Transform3D transform_local = p_body.get_shape_transform_scaled(i);
		const Transform3D transform_com_local = transform_local.translated_local(com_scaled);
		const Transform3D transform_com = body_transform * transform_com_local;
		const Transform3D transform_com_unscaled = Math::decomposed(transform_com, scale);

		float shape_safe_fraction = 1.0f;
		float shape_unsafe_fraction = 1.0f;

		collided |= _cast_motion_impl(
			*jolt_shape,
			transform_com_unscaled,
			scale,
			p_motion,
			false,
			settings,
			motion_filter,
			motion_filter,
			motion_filter,
			motion_filter,
			shape_safe_fraction,
			shape_unsafe_fraction
		);

		p_safe_fraction = min(p_safe_fraction, shape_safe_fraction);
		p_unsafe_fraction = min(p_unsafe_fraction, shape_unsafe_fraction);
	}

	return collided;
}

bool JoltPhysicsDirectSpaceState3D::_body_motion_collide(
	const JoltBodyImpl3D& p_body,
	const Transform3D& p_transform,
	float p_distance,
	float p_margin,
	int32_t p_max_collisions,
	PhysicsServer3DExtensionMotionResult* p_result
) const {
	const JPH::Shape* jolt_shape = p_body.get_jolt_shape();

	const Vector3 com_scaled = to_godot(jolt_shape->GetCenterOfMass());
	const Transform3D transform_com = p_transform.translated_local(com_scaled);

	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mMaxSeparationDistance = p_margin;

	const Vector3& base_offset = transform_com.origin;

	const JoltMotionFilter3D motion_filter(p_body);

	JoltQueryCollectorClosestMulti<JPH::CollideShapeCollector, 32> collector(p_max_collisions);

	space->get_narrow_phase_query().CollideShape(
		jolt_shape,
		JPH::Vec3::sReplicate(1.0f),
		to_jolt(transform_com),
		settings,
		to_jolt(base_offset),
		collector,
		motion_filter,
		motion_filter,
		motion_filter,
		motion_filter
	);

	const bool collided = collector.had_hit();

	if (!collided || p_result == nullptr) {
		return collided;
	}

	// HACK(mihe): Without this minimum contact depth we can sometimes end up with very shallow
	// contacts that end up affecting the outcome of things like `floor_block_on_wall`, where after
	// one of the recovery iterations (in Godot, not here) we can still find ourselves penetrating a
	// wall ever so slightly, which `move_and_slide` will interpret as being trapped in a corner and
	// stop the character altogether. We still need distances smaller than this (like none at all)
	// to actually emit contacts though, so we clamp it by the distance moved.
	const float min_contact_depth = min(0.0001f, p_distance);

	int32_t count = 0;

	for (int32_t i = 0; i < collector.get_hit_count(); ++i) {
		const JPH::CollideShapeResult& hit = collector.get_hit(i);

		const float penetration_depth = hit.mPenetrationDepth + p_margin - min_contact_depth;

		if (penetration_depth <= 0.0f) {
			continue;
		}

		const JoltReadableBody3D collider_jolt_body = space->read_body(hit.mBodyID2);
		const JoltObjectImpl3D* collider = collider_jolt_body.as_object();
		ERR_FAIL_NULL_D(collider);

		const Vector3 position = base_offset + to_godot(hit.mContactPointOn2);

		const int32_t local_shape = p_body.find_shape_index(hit.mSubShapeID1);
		ERR_FAIL_COND_D(local_shape == -1);

		const int32_t collider_shape = collider->find_shape_index(hit.mSubShapeID2);
		ERR_FAIL_COND_D(collider_shape == -1);

		PhysicsServer3DExtensionMotionCollision& collision = p_result->collisions[count++];

		collision.position = position;
		collision.normal = to_godot(-hit.mPenetrationAxis.Normalized());
		collision.collider_velocity = collider->get_velocity_at_position(position);
		collision.collider_angular_velocity = collider->get_angular_velocity();
		collision.depth = penetration_depth;
		collision.local_shape = local_shape;
		collision.collider_id = collider->get_instance_id();
		collision.collider = collider->get_rid();
		collision.collider_shape = collider_shape;
	}

	p_result->collision_count = count;

	return count > 0;
}
