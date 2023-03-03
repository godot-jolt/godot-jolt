#include "jolt_contact_listener.hpp"

#include "jolt_area_3d.hpp"
#include "jolt_body_3d.hpp"
#include "jolt_space_3d.hpp"

namespace {

JPH::SubShapeIDPair pair_contact(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold
) {
	return {p_body1.GetID(), p_manifold.mSubShapeID1, p_body2.GetID(), p_manifold.mSubShapeID2};
}

} // namespace

void JoltContactListener::listen_for(JoltCollisionObject3D* p_object) {
	listening_for.insert(p_object->get_jolt_id());
}

void JoltContactListener::pre_step() {
	listening_for.clear();
	debug_contact_count = 0;
}

void JoltContactListener::post_step() {
	flush_contacts();
	flush_area_shifts();
	flush_area_exits();
	flush_area_enters();
}

void JoltContactListener::OnContactAdded(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	JPH::ContactSettings& p_settings
) {
#if DEBUG_ENABLED
	add_debug_contacts(p_manifold);
#endif // DEBUG_ENABLED

	if (!is_listening_for(p_body1) && !is_listening_for(p_body2)) {
		return;
	}

	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		const JPH::SubShapeIDPair shape_pair = pair_contact(p_body1, p_body2, p_manifold);

		const MutexLock write_lock(write_mutex);

		area_enters.insert(shape_pair);
		area_overlaps.insert(shape_pair);
	} else {
		update_contacts(p_body1, p_body2, p_manifold, p_settings);
	}
}

void JoltContactListener::OnContactPersisted(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	JPH::ContactSettings& p_settings
) {
#if DEBUG_ENABLED
	add_debug_contacts(p_manifold);
#endif // DEBUG_ENABLED

	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		return;
	}

	if (!is_listening_for(p_body1) && !is_listening_for(p_body2)) {
		return;
	}

	update_contacts(p_body1, p_body2, p_manifold, p_settings);
}

void JoltContactListener::OnContactRemoved(const JPH::SubShapeIDPair& p_shape_pair) {
	const MutexLock write_lock(write_mutex);

	if (area_overlaps.erase(p_shape_pair)) {
		area_exits.insert(p_shape_pair);
	} else {
		manifolds_by_shape_pair.erase(p_shape_pair);
	}
}

bool JoltContactListener::is_listening_for(const JPH::Body& p_body) {
	return listening_for.has(p_body.GetID());
}

void JoltContactListener::update_contacts(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	JPH::ContactSettings& p_settings
) {
	const JPH::SubShapeIDPair shape_pair = pair_contact(p_body1, p_body2, p_manifold);

	const MutexLock write_lock(write_mutex);

	Manifold& manifold = manifolds_by_shape_pair[shape_pair];

	const JPH::uint contact_count = p_manifold.mRelativeContactPointsOn1.size();

	manifold.contacts1.reserve((int32_t)contact_count);
	manifold.contacts2.reserve((int32_t)contact_count);
	manifold.depth = p_manifold.mPenetrationDepth;

	const JPH::Vec3 body_position1 = p_body1.GetPosition();
	const JPH::Vec3 body_position2 = p_body2.GetPosition();

	JPH::CollisionEstimationResult collision;

	JPH::EstimateCollisionResponse(
		p_body1,
		p_body2,
		p_manifold,
		collision,
		p_settings.mCombinedFriction,
		p_settings.mCombinedRestitution
	);

	for (JPH::uint i = 0; i < contact_count; ++i) {
		Contact& contact1 = manifold.contacts1.emplace_back();
		Contact& contact2 = manifold.contacts2.emplace_back();

		const JPH::Vec3& relative_point1 = p_manifold.mRelativeContactPointsOn1[i];
		const JPH::Vec3& relative_point2 = p_manifold.mRelativeContactPointsOn2[i];

		const JPH::Vec3 world_point1 = p_manifold.mBaseOffset + relative_point1;
		const JPH::Vec3 world_point2 = p_manifold.mBaseOffset + relative_point2;

		const JPH::CollisionEstimationResult::Impulse& impulse = collision.mImpulses[i];

		const JPH::Vec3 contact_impulse = p_manifold.mWorldSpaceNormal * impulse.mContactImpulse;
		const JPH::Vec3 friction_impulse1 = collision.mTangent1 * impulse.mFrictionImpulse1;
		const JPH::Vec3 friction_impulse2 = collision.mTangent2 * impulse.mFrictionImpulse2;
		const JPH::Vec3 combined_impulse = contact_impulse + friction_impulse1 + friction_impulse2;

		contact1.normal = -p_manifold.mWorldSpaceNormal;
		contact1.point_self = world_point1 - body_position1;
		contact1.point_other = world_point1 - body_position2;
		contact1.velocity_other = p_body2.GetPointVelocity(world_point1);
		contact1.impulse = -combined_impulse;

		contact2.normal = p_manifold.mWorldSpaceNormal;
		contact2.point_self = world_point2 - body_position2;
		contact2.point_other = world_point2 - body_position1;
		contact2.velocity_other = p_body1.GetPointVelocity(world_point2);
		contact2.impulse = combined_impulse;
	}
}

void JoltContactListener::flush_contacts() {
	for (auto&& [shape_pair, manifold] : manifolds_by_shape_pair) {
		const JPH::BodyID body_ids[] = {shape_pair.GetBody1ID(), shape_pair.GetBody2ID()};

		const JoltReadableBodies3D jolt_bodies =
			space->read_bodies(body_ids, count_of(body_ids), false);

		JoltBody3D* body1 = jolt_bodies[0].as_body();
		ERR_FAIL_NULL(body1);

		JoltBody3D* body2 = jolt_bodies[1].as_body();
		ERR_FAIL_NULL(body2);

		const JPH::Shape& shape1 = *body1->get_jolt_shape();
		const JPH::Shape& shape2 = *body2->get_jolt_shape();

		const JPH::SubShapeID& sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID& sub_shape_id2 = shape_pair.GetSubShapeID2();

		const auto shape_instance_id1 = (uint32_t)shape1.GetSubShapeUserData(sub_shape_id1);
		const auto shape_instance_id2 = (uint32_t)shape2.GetSubShapeUserData(sub_shape_id2);

		const int32_t shape_index1 = body1->find_shape_index(shape_instance_id1);
		const int32_t shape_index2 = body2->find_shape_index(shape_instance_id2);

		for (const Contact& contact : manifold.contacts1) {
			body1->add_contact(
				body2,
				manifold.depth,
				shape_index1,
				shape_index2,
				to_godot(contact.normal),
				to_godot(contact.point_self),
				to_godot(contact.point_other),
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
				to_godot(contact.velocity_other),
				to_godot(contact.impulse)
			);
		}

		manifold.contacts1.clear();
		manifold.contacts2.clear();
	}
}

void JoltContactListener::flush_area_enters() {
	for (const JPH::SubShapeIDPair& shape_pair : area_enters) {
		const JPH::BodyID& body_id1 = shape_pair.GetBody1ID();
		const JPH::BodyID& body_id2 = shape_pair.GetBody2ID();

		const JPH::SubShapeID& sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID& sub_shape_id2 = shape_pair.GetSubShapeID2();

		const JPH::BodyID body_ids[] = {body_id1, body_id2};

		const JoltReadableBodies3D jolt_bodies =
			space->read_bodies(body_ids, count_of(body_ids), false);

		const JoltReadableBody3D jolt_body1 = jolt_bodies[0];
		const JoltReadableBody3D jolt_body2 = jolt_bodies[1];

		if (jolt_body1.is_invalid() || jolt_body2.is_invalid()) {
			continue;
		}

		JoltArea3D* area1 = jolt_body1.as_area();
		JoltArea3D* area2 = jolt_body2.as_area();

		if (area1 && area2) {
			if (area2->is_monitorable()) {
				area1->area_shape_entered(body_id2, sub_shape_id2, sub_shape_id1);
			}

			if (area1->is_monitorable()) {
				area2->area_shape_entered(body_id1, sub_shape_id1, sub_shape_id2);
			}
		} else if (area1 && !area2) {
			area1->body_shape_entered(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (!area1 && area2) {
			area2->body_shape_entered(body_id1, sub_shape_id1, sub_shape_id2);
		}
	}

	area_enters.clear();
}

void JoltContactListener::flush_area_shifts() {
	for (const JPH::SubShapeIDPair& shape_pair : area_overlaps) {
		auto is_shifted = [&](const JPH::BodyID& p_body_id, const JPH::SubShapeID& p_sub_shape_id) {
			const JoltReadableBody3D jolt_body = space->read_body(p_body_id, false);
			const JoltCollisionObject3D* object = jolt_body.as_object();
			ERR_FAIL_NULL_V(object, false);

			if (!object->get_previous_jolt_shape()) {
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

void JoltContactListener::flush_area_exits() {
	for (const JPH::SubShapeIDPair& shape_pair : area_exits) {
		const JPH::BodyID& body_id1 = shape_pair.GetBody1ID();
		const JPH::BodyID& body_id2 = shape_pair.GetBody2ID();

		const JPH::SubShapeID& sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID& sub_shape_id2 = shape_pair.GetSubShapeID2();

		const JPH::BodyID body_ids[] = {body_id1, body_id2};

		const JoltReadableBodies3D jolt_bodies =
			space->read_bodies(body_ids, count_of(body_ids), false);

		const JoltReadableBody3D jolt_body1 = jolt_bodies[0];
		const JoltReadableBody3D jolt_body2 = jolt_bodies[1];

		JoltArea3D* area1 = jolt_body1.as_area();
		JoltArea3D* area2 = jolt_body2.as_area();

		JoltBody3D* body1 = jolt_body1.as_body();
		JoltBody3D* body2 = jolt_body2.as_body();

		if (area1 && area2) {
			area1->area_shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
			area2->area_shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		} else if (area1 && body2) {
			area1->body_shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (body1 && area2) {
			area2->body_shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		} else if (area1) {
			area1->shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area2) {
			area2->shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		}
	}

	area_exits.clear();
}

#if DEBUG_ENABLED

void JoltContactListener::add_debug_contacts(const JPH::ContactManifold& p_manifold) {
	const int64_t max_count = debug_contacts.size();

	if (max_count == 0) {
		return;
	}

	const auto additional_pairs = (int32_t)p_manifold.mRelativeContactPointsOn1.size();
	const int32_t additional_contacts = additional_pairs * 2;

	int32_t current_count = debug_contact_count;
	bool exchanged = false;

	do {
		const int32_t new_count = current_count + additional_contacts;

		if (new_count > max_count) {
			return;
		}

		exchanged = debug_contact_count.compare_exchange_weak(current_count, new_count);
	} while (!exchanged);

	for (int32_t i = 0; i < additional_pairs; ++i) {
		const int32_t pair_index = current_count + i * 2;

		debug_contacts[pair_index + 0] =
			to_godot(p_manifold.GetWorldSpaceContactPointOn1((JPH::uint)i));

		debug_contacts[pair_index + 1] =
			to_godot(p_manifold.GetWorldSpaceContactPointOn2((JPH::uint)i));
	}
}

#endif // DEBUG_ENABLED
