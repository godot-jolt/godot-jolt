#include "jolt_contact_listener_3d.hpp"

#include "objects/jolt_area_impl_3d.hpp"
#include "objects/jolt_body_impl_3d.hpp"
#include "servers/jolt_project_settings.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

JPH::SubShapeIDPair pair_contact(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold
) {
	return {p_body1.GetID(), p_manifold.mSubShapeID1, p_body2.GetID(), p_manifold.mSubShapeID2};
}

} // namespace

void JoltContactListener3D::listen_for(JoltObjectImpl3D* p_object) {
	listening_for.insert(p_object->get_jolt_id());
}

void JoltContactListener3D::pre_step() {
	listening_for.clear();

#ifdef GDJ_CONFIG_EDITOR
	debug_contact_count = 0;
#endif // GDJ_CONFIG_EDITOR
}

void JoltContactListener3D::post_step() {
	flush_contacts();
	flush_area_shifts();
	flush_area_exits();
	flush_area_enters();
}

void JoltContactListener3D::OnContactAdded(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	JPH::ContactSettings& p_settings
) {
	try_override_collision_response(p_body1, p_body2, p_settings);
	try_apply_surface_velocities(p_body1, p_body2, p_settings);
	try_add_contacts(p_body1, p_body2, p_manifold, p_settings);
	try_add_area_overlap(p_body1, p_body2, p_manifold);

#ifdef GDJ_CONFIG_EDITOR
	try_add_debug_contacts(p_manifold);
#endif // GDJ_CONFIG_EDITOR
}

void JoltContactListener3D::OnContactPersisted(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	JPH::ContactSettings& p_settings
) {
	try_override_collision_response(p_body1, p_body2, p_settings);
	try_apply_surface_velocities(p_body1, p_body2, p_settings);
	try_add_contacts(p_body1, p_body2, p_manifold, p_settings);

#ifdef GDJ_CONFIG_EDITOR
	try_add_debug_contacts(p_manifold);
#endif // GDJ_CONFIG_EDITOR
}

void JoltContactListener3D::OnContactRemoved(const JPH::SubShapeIDPair& p_shape_pair) {
	if (!try_remove_contacts(p_shape_pair)) {
		try_remove_area_overlap(p_shape_pair);
	}
}

bool JoltContactListener3D::is_listening_for(const JPH::Body& p_body) const {
	return listening_for.has(p_body.GetID());
}

bool JoltContactListener3D::try_override_collision_response(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	JPH::ContactSettings& p_settings
) {
	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		return false;
	}

	if (!p_body1.IsDynamic() && !p_body2.IsDynamic()) {
		return false;
	}

	JPH::BroadPhaseLayer broad_phase_layer1 = {};
	uint32_t collision_layer1 = 0;
	uint32_t collision_mask1 = 0;

	space->map_from_object_layer(
		p_body1.GetObjectLayer(),
		broad_phase_layer1,
		collision_layer1,
		collision_mask1
	);

	JPH::BroadPhaseLayer broad_phase_layer2 = {};
	uint32_t collision_layer2 = 0;
	uint32_t collision_mask2 = 0;

	space->map_from_object_layer(
		p_body2.GetObjectLayer(),
		broad_phase_layer2,
		collision_layer2,
		collision_mask2
	);

	const bool first_scans_second = (collision_mask1 & collision_layer2) != 0;
	const bool second_scans_first = (collision_mask2 & collision_layer1) != 0;

	if (first_scans_second && !second_scans_first) {
		p_settings.mInvMassScale2 = 0.0f;
		p_settings.mInvInertiaScale2 = 0.0f;
	} else if (second_scans_first && !first_scans_second) {
		p_settings.mInvMassScale1 = 0.0f;
		p_settings.mInvInertiaScale1 = 0.0f;
	}

	return true;
}

bool JoltContactListener3D::try_apply_surface_velocities(
	const JPH::Body& p_jolt_body1,
	const JPH::Body& p_jolt_body2,
	JPH::ContactSettings& p_settings
) {
	if (p_jolt_body1.IsSensor() || p_jolt_body2.IsSensor()) {
		return false;
	}

	const bool supports_surface_velocity1 = !p_jolt_body1.IsDynamic();
	const bool supports_surface_velocity2 = !p_jolt_body2.IsDynamic();

	if (supports_surface_velocity1 == supports_surface_velocity2) {
		return false;
	}

	const auto* body1 = reinterpret_cast<JoltBodyImpl3D*>(p_jolt_body1.GetUserData());
	const auto* body2 = reinterpret_cast<JoltBodyImpl3D*>(p_jolt_body2.GetUserData());

	const bool has_surface_velocity1 = supports_surface_velocity1 &&
		(body1->get_linear_surface_velocity() != Vector3() ||
		 body1->get_angular_surface_velocity() != Vector3());

	const bool has_surface_velocity2 = supports_surface_velocity2 &&
		(body2->get_linear_surface_velocity() != Vector3() ||
		 body2->get_angular_surface_velocity() != Vector3());

	if (has_surface_velocity1 == has_surface_velocity2) {
		return false;
	}

	const JPH::Vec3 linear_velocity1 = to_jolt(body1->get_linear_surface_velocity());
	const JPH::Vec3 angular_velocity1 = to_jolt(body1->get_angular_surface_velocity());

	const JPH::Vec3 linear_velocity2 = to_jolt(body2->get_linear_surface_velocity());
	const JPH::Vec3 angular_velocity2 = to_jolt(body2->get_angular_surface_velocity());

	const JPH::Vec3 com1 = p_jolt_body1.GetCenterOfMassPosition();
	const JPH::Vec3 com2 = p_jolt_body2.GetCenterOfMassPosition();
	const JPH::Vec3 rel_com2 = com2 - com1;

	const JPH::Vec3 angular_linear_velocity2 = rel_com2.Cross(angular_velocity2);
	const JPH::Vec3 total_linear_velocity2 = linear_velocity2 + angular_linear_velocity2;

	p_settings.mRelativeSurfaceVelocity = total_linear_velocity2 - linear_velocity1;
	p_settings.mRelativeAngularSurfaceVelocity = angular_velocity2 - angular_velocity1;

	return true;
}

bool JoltContactListener3D::try_add_contacts(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	JPH::ContactSettings& p_settings
) {
	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		return false;
	}

	if (!is_listening_for(p_body1) && !is_listening_for(p_body2)) {
		return false;
	}

	const JPH::SubShapeIDPair shape_pair = pair_contact(p_body1, p_body2, p_manifold);

	auto& manifold = [&]() -> Manifold& {
		const MutexLock write_lock(write_mutex);
		return manifolds_by_shape_pair[shape_pair];
	}();

	const JPH::uint contact_count = p_manifold.mRelativeContactPointsOn1.size();

	manifold.contacts1.reserve((int32_t)contact_count);
	manifold.contacts2.reserve((int32_t)contact_count);
	manifold.depth = p_manifold.mPenetrationDepth;

	JPH::CollisionEstimationResult collision;

	JPH::EstimateCollisionResponse(
		p_body1,
		p_body2,
		p_manifold,
		collision,
		p_settings.mCombinedFriction,
		p_settings.mCombinedRestitution,
		JoltProjectSettings::get_bounce_velocity_threshold(),
		5
	);

	for (JPH::uint i = 0; i < contact_count; ++i) {
		Contact& contact1 = manifold.contacts1.emplace_back();
		Contact& contact2 = manifold.contacts2.emplace_back();

		const JPH::Vec3& relative_point1 = p_manifold.mRelativeContactPointsOn1[i];
		const JPH::Vec3& relative_point2 = p_manifold.mRelativeContactPointsOn2[i];

		const JPH::Vec3 world_point1 = p_manifold.mBaseOffset + relative_point1;
		const JPH::Vec3 world_point2 = p_manifold.mBaseOffset + relative_point2;

		const JPH::Vec3 velocity1 = p_body1.GetPointVelocity(world_point1);
		const JPH::Vec3 velocity2 = p_body2.GetPointVelocity(world_point2);

		const JPH::CollisionEstimationResult::Impulse& impulse = collision.mImpulses[i];

		const JPH::Vec3 contact_impulse = p_manifold.mWorldSpaceNormal * impulse.mContactImpulse;
		const JPH::Vec3 friction_impulse1 = collision.mTangent1 * impulse.mFrictionImpulse1;
		const JPH::Vec3 friction_impulse2 = collision.mTangent2 * impulse.mFrictionImpulse2;
		const JPH::Vec3 combined_impulse = contact_impulse + friction_impulse1 + friction_impulse2;

		contact1.normal = -p_manifold.mWorldSpaceNormal;
		contact1.point_self = world_point1;
		contact1.point_other = world_point2;
		contact1.velocity_self = velocity1;
		contact1.velocity_other = velocity2;
		contact1.impulse = -combined_impulse;

		contact2.normal = p_manifold.mWorldSpaceNormal;
		contact2.point_self = world_point2;
		contact2.point_other = world_point1;
		contact2.velocity_self = velocity2;
		contact2.velocity_other = velocity1;
		contact2.impulse = combined_impulse;
	}

	return true;
}

bool JoltContactListener3D::try_add_area_overlap(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold
) {
	if (!p_body1.IsSensor() && !p_body2.IsSensor()) {
		return false;
	}

	if (!is_listening_for(p_body1) && !is_listening_for(p_body2)) {
		return false;
	}

	const JPH::SubShapeIDPair shape_pair = pair_contact(p_body1, p_body2, p_manifold);

	const MutexLock write_lock(write_mutex);

	area_enters.insert(shape_pair);
	area_overlaps.insert(shape_pair);

	return true;
}

bool JoltContactListener3D::try_remove_contacts(const JPH::SubShapeIDPair& p_shape_pair) {
	const MutexLock write_lock(write_mutex);

	return manifolds_by_shape_pair.erase(p_shape_pair);
}

bool JoltContactListener3D::try_remove_area_overlap(const JPH::SubShapeIDPair& p_shape_pair) {
	const MutexLock write_lock(write_mutex);

	if (!area_overlaps.erase(p_shape_pair)) {
		return false;
	}

	area_exits.insert(p_shape_pair);

	return true;
}

#ifdef GDJ_CONFIG_EDITOR

bool JoltContactListener3D::try_add_debug_contacts(const JPH::ContactManifold& p_manifold) {
	const int64_t max_count = debug_contacts.size();

	if (max_count == 0) {
		return false;
	}

	const auto additional_pairs = (int32_t)p_manifold.mRelativeContactPointsOn1.size();
	const int32_t additional_contacts = additional_pairs * 2;

	int32_t current_count = debug_contact_count;
	bool exchanged = false;

	do {
		const int32_t new_count = current_count + additional_contacts;

		if (new_count > max_count) {
			return false;
		}

		exchanged = debug_contact_count.compare_exchange_weak(current_count, new_count);
	} while (!exchanged);

	for (int32_t i = 0; i < additional_pairs; ++i) {
		const int32_t pair_index = current_count + i * 2;

		const JPH::Vec3 point_on_1 = p_manifold.GetWorldSpaceContactPointOn1((JPH::uint)i);
		const JPH::Vec3 point_on_2 = p_manifold.GetWorldSpaceContactPointOn2((JPH::uint)i);

		debug_contacts[pair_index + 0] = to_godot(point_on_1);
		debug_contacts[pair_index + 1] = to_godot(point_on_2);
	}

	return true;
}

#endif // GDJ_CONFIG_EDITOR

void JoltContactListener3D::flush_contacts() {
	for (auto&& [shape_pair, manifold] : manifolds_by_shape_pair) {
		const JPH::BodyID body_ids[] = {shape_pair.GetBody1ID(), shape_pair.GetBody2ID()};

		const JoltReadableBodies3D jolt_bodies = space->read_bodies(
			body_ids,
			count_of(body_ids),
			false
		);

		JoltBodyImpl3D* body1 = jolt_bodies[0].as_body();
		ERR_FAIL_NULL(body1);

		JoltBodyImpl3D* body2 = jolt_bodies[1].as_body();
		ERR_FAIL_NULL(body2);

		const int32_t shape_index1 = body1->find_shape_index(shape_pair.GetSubShapeID1());
		const int32_t shape_index2 = body2->find_shape_index(shape_pair.GetSubShapeID2());

		for (const Contact& contact : manifold.contacts1) {
			body1->add_contact(
				body2,
				manifold.depth,
				shape_index1,
				shape_index2,
				to_godot(contact.normal),
				to_godot(contact.point_self),
				to_godot(contact.point_other),
				to_godot(contact.velocity_self),
				to_godot(contact.velocity_other),
				to_godot(contact.impulse)
			);
		}

		for (const Contact& contact : manifold.contacts2) {
			body2->add_contact(
				body1,
				manifold.depth,
				shape_index2,
				shape_index1,
				to_godot(contact.normal),
				to_godot(contact.point_self),
				to_godot(contact.point_other),
				to_godot(contact.velocity_self),
				to_godot(contact.velocity_other),
				to_godot(contact.impulse)
			);
		}

		manifold.contacts1.clear();
		manifold.contacts2.clear();
	}
}

void JoltContactListener3D::flush_area_enters() {
	for (const JPH::SubShapeIDPair& shape_pair : area_enters) {
		const JPH::BodyID& body_id1 = shape_pair.GetBody1ID();
		const JPH::BodyID& body_id2 = shape_pair.GetBody2ID();

		const JPH::SubShapeID& sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID& sub_shape_id2 = shape_pair.GetSubShapeID2();

		const JPH::BodyID body_ids[] = {body_id1, body_id2};

		const JoltReadableBodies3D jolt_bodies = space->read_bodies(
			body_ids,
			count_of(body_ids),
			false
		);

		const JoltReadableBody3D jolt_body1 = jolt_bodies[0];
		const JoltReadableBody3D jolt_body2 = jolt_bodies[1];

		if (jolt_body1.is_invalid() || jolt_body2.is_invalid()) {
			continue;
		}

		JoltAreaImpl3D* area1 = jolt_body1.as_area();
		JoltAreaImpl3D* area2 = jolt_body2.as_area();

		if (area1 != nullptr && area2 != nullptr) {
			if (area2->is_monitorable()) {
				area1->area_shape_entered(body_id2, sub_shape_id2, sub_shape_id1);
			}

			if (area1->is_monitorable()) {
				area2->area_shape_entered(body_id1, sub_shape_id1, sub_shape_id2);
			}
		} else if (area1 != nullptr && area2 == nullptr) {
			area1->body_shape_entered(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area1 == nullptr && area2 != nullptr) {
			area2->body_shape_entered(body_id1, sub_shape_id1, sub_shape_id2);
		}
	}

	area_enters.clear();
}

void JoltContactListener3D::flush_area_shifts() {
	for (const JPH::SubShapeIDPair& shape_pair : area_overlaps) {
		auto is_shifted = [&](const JPH::BodyID& p_body_id, const JPH::SubShapeID& p_sub_shape_id) {
			const JoltReadableBody3D jolt_body = space->read_body(p_body_id, false);
			const JoltObjectImpl3D* object = jolt_body.as_object();
			ERR_FAIL_NULL_V(object, false);

			if (object->get_previous_jolt_shape() == nullptr) {
				return false;
			}

			const JPH::Shape& current_shape = *object->get_jolt_shape();
			const JPH::Shape& previous_shape = *object->get_previous_jolt_shape();

			const auto current_id = (uint32_t)current_shape.GetSubShapeUserData(p_sub_shape_id);
			const auto previous_id = (uint32_t)previous_shape.GetSubShapeUserData(p_sub_shape_id);

			return current_id != previous_id;
		};

		if (is_shifted(shape_pair.GetBody1ID(), shape_pair.GetSubShapeID1()) ||
			is_shifted(shape_pair.GetBody2ID(), shape_pair.GetSubShapeID2()))
		{
			area_enters.insert(shape_pair);
			area_exits.insert(shape_pair);
		}
	}
}

void JoltContactListener3D::flush_area_exits() {
	for (const JPH::SubShapeIDPair& shape_pair : area_exits) {
		const JPH::BodyID& body_id1 = shape_pair.GetBody1ID();
		const JPH::BodyID& body_id2 = shape_pair.GetBody2ID();

		const JPH::SubShapeID& sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID& sub_shape_id2 = shape_pair.GetSubShapeID2();

		const JPH::BodyID body_ids[] = {body_id1, body_id2};

		const JoltReadableBodies3D jolt_bodies = space->read_bodies(
			body_ids,
			count_of(body_ids),
			false
		);

		const JoltReadableBody3D jolt_body1 = jolt_bodies[0];
		const JoltReadableBody3D jolt_body2 = jolt_bodies[1];

		JoltAreaImpl3D* area1 = jolt_body1.as_area();
		JoltAreaImpl3D* area2 = jolt_body2.as_area();

		const JoltBodyImpl3D* body1 = jolt_body1.as_body();
		const JoltBodyImpl3D* body2 = jolt_body2.as_body();

		if (area1 != nullptr && area2 != nullptr) {
			area1->area_shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
			area2->area_shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		} else if (area1 != nullptr && body2 != nullptr) {
			area1->body_shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (body1 != nullptr && area2 != nullptr) {
			area2->body_shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		} else if (area1 != nullptr) {
			area1->shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area2 != nullptr) {
			area2->shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		}
	}

	area_exits.clear();
}
