#include "jolt_motion_filter_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "objects/jolt_collision_object_3d.hpp"
#include "servers/jolt_physics_server_3d.hpp"
#include "shapes/jolt_motion_shape.hpp"
#include "shapes/jolt_shape_3d.hpp"
#include "shapes/jolt_shape_type.hpp"
#include "spaces/jolt_broad_phase_layer.hpp"
#include "spaces/jolt_space_3d.hpp"

JoltMotionFilter3D::JoltMotionFilter3D(const JoltBodyImpl3D& p_body, bool p_collide_separation_ray)
	: physics_server(*static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton()))
	, body(p_body)
	, collide_separation_ray(p_collide_separation_ray) { }

bool JoltMotionFilter3D::ShouldCollide(JPH::BroadPhaseLayer p_broad_phase_layer) const {
	const auto broad_phase_layer = (JPH::BroadPhaseLayer::Type)p_broad_phase_layer;

	switch (broad_phase_layer) {
		case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::BODY_STATIC:
		case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::BODY_DYNAMIC: {
			return true;
		} break;
		case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::AREA_DETECTABLE:
		case (JPH::BroadPhaseLayer::Type)JoltBroadPhaseLayer::AREA_UNDETECTABLE: {
			return false;
		} break;
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled broad phase layer: '%d'", broad_phase_layer));
		}
	}
}

bool JoltMotionFilter3D::ShouldCollide(JPH::ObjectLayer p_object_layer) const {
	JPH::BroadPhaseLayer object_broad_phase_layer = {};
	uint32_t object_collision_layer = 0;
	uint32_t object_collision_mask = 0;

	body.get_space()->map_from_object_layer(
		p_object_layer,
		object_broad_phase_layer,
		object_collision_layer,
		object_collision_mask
	);

	return (body.get_collision_mask() & object_collision_layer) != 0;
}

bool JoltMotionFilter3D::ShouldCollide(const JPH::BodyID& p_body_id) const {
	return p_body_id != body.get_jolt_id();
}

bool JoltMotionFilter3D::ShouldCollideLocked(const JPH::Body& p_body) const {
	const auto* object = reinterpret_cast<const JoltObjectImpl3D*>(p_body.GetUserData());

	return !physics_server.body_test_motion_is_excluding_object(object->get_instance_id()) &&
		!physics_server.body_test_motion_is_excluding_body(object->get_rid());
}

bool JoltMotionFilter3D::ShouldCollide(
	[[maybe_unused]] const JPH::Shape* p_shape2,
	[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id2
) const {
	return true;
}

bool JoltMotionFilter3D::ShouldCollide(
	const JPH::Shape* p_shape1,
	[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id1,
	[[maybe_unused]] const JPH::Shape* p_shape2,
	[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id2
) const {
	if (collide_separation_ray) {
		return true;
	}

	const auto* motion_shape1 = static_cast<const JoltCustomMotionShape*>(p_shape1);
	const JPH::ConvexShape& actual_shape1 = motion_shape1->get_inner_shape();

	return actual_shape1.GetSubType() != JoltShapeSubType::RAY;
}
