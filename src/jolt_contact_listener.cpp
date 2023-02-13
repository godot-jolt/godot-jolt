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

float calculate_contact_impulse(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::Vec3Arg p_contact_point,
	const JPH::Vec3Arg p_contact_normal
) {
	// HACK(mihe): According to Jolt's documentation, the constraint solver hasn't run when the
	// contacts are added, which means we have no way of getting this impulse through Jolt, so we
	// have to calculate it ourselves instead.
	//
	// I frankly don't know if these calculations are correct within this context. They don't take
	// friction into account, so there's at the very least some room for improvement. I suspect
	// there's also some time component missing. If you can see a better way of doing this, please
	// make it known.
	//
	// Courtesy of: research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials

	const float restitution = max(p_body1.GetRestitution(), p_body2.GetRestitution());

	const JPH::Vec3 contact_from_com1 = p_contact_point - p_body1.GetCenterOfMassPosition();
	const JPH::Vec3 contact_from_com2 = p_contact_point - p_body2.GetCenterOfMassPosition();

	const JPH::MotionProperties& motion1 = *p_body1.GetMotionPropertiesUnchecked();
	const JPH::MotionProperties& motion2 = *p_body2.GetMotionPropertiesUnchecked();

	const JPH::Vec3 angular_velocity1 = motion1.GetAngularVelocity().Cross(contact_from_com1);
	const JPH::Vec3 angular_velocity2 = motion2.GetAngularVelocity().Cross(contact_from_com2);

	const JPH::Vec3 contact_velocity1 = motion1.GetLinearVelocity() + angular_velocity1;
	const JPH::Vec3 contact_velocity2 = motion2.GetLinearVelocity() + angular_velocity2;

	const JPH::Vec3 relative_velocity = contact_velocity2 - contact_velocity1;

	const float impulse_force = relative_velocity.Dot(p_contact_normal);

	const bool is_dynamic1 = p_body1.IsDynamic();
	const bool is_dynamic2 = p_body2.IsDynamic();

	const float inverse_mass1 = is_dynamic1 ? motion1.GetInverseMassUnchecked() : 0.0f;
	const float inverse_mass2 = is_dynamic2 ? motion2.GetInverseMassUnchecked() : 0.0f;

	const float total_mass = inverse_mass1 + inverse_mass2;

	JPH::Vec3 inertia1 = JPH::Vec3::sZero();
	JPH::Vec3 inertia2 = JPH::Vec3::sZero();

	// clang-format off

	if (is_dynamic1) {
		inertia1 = motion1.MultiplyWorldSpaceInverseInertiaByVector(
			p_body1.GetRotation(),
			contact_from_com1.Cross(p_contact_normal)
		).Cross(contact_from_com1);
	}

	if (is_dynamic2) {
		inertia2 = motion2.MultiplyWorldSpaceInverseInertiaByVector(
			p_body2.GetRotation(),
			contact_from_com2.Cross(p_contact_normal)
		).Cross(contact_from_com2);
	}

	// clang-format on

	const float angular_effect = (inertia1 + inertia2).Dot(p_contact_normal);

	return ((1.0f + restitution) * impulse_force) / (total_mass + angular_effect);
}

} // namespace

void JoltContactListener::listen_for(JoltCollisionObject3D* p_object) {
	listening_for.insert(p_object->get_jolt_id());
}

void JoltContactListener::pre_step() {
	listening_for.clear();
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
	[[maybe_unused]] JPH::ContactSettings& p_settings
) {
	if (!is_listening_for(p_body1) && !is_listening_for(p_body2)) {
		return;
	}

	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		const JPH::SubShapeIDPair shape_pair = pair_contact(p_body1, p_body2, p_manifold);

		const MutexLock write_lock(write_mutex);

		area_enters.insert(shape_pair);
		area_overlaps.insert(shape_pair);
	} else {
		update_contacts(p_body1, p_body2, p_manifold);
	}
}

void JoltContactListener::OnContactPersisted(
	const JPH::Body& p_body1,
	const JPH::Body& p_body2,
	const JPH::ContactManifold& p_manifold,
	[[maybe_unused]] JPH::ContactSettings& p_settings
) {
	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		return;
	}

	if (!is_listening_for(p_body1) && !is_listening_for(p_body2)) {
		return;
	}

	update_contacts(p_body1, p_body2, p_manifold);
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
	const JPH::ContactManifold& p_manifold
) {
	const JPH::SubShapeIDPair shape_pair = pair_contact(p_body1, p_body2, p_manifold);

	const MutexLock write_lock(write_mutex);

	Manifold& manifold = manifolds_by_shape_pair[shape_pair];

	manifold.contacts1.reserve((int32_t)p_manifold.mRelativeContactPointsOn1.size());
	manifold.contacts2.reserve((int32_t)p_manifold.mRelativeContactPointsOn2.size());
	manifold.depth = p_manifold.mPenetrationDepth;

	const JPH::Vec3 body_position1 = p_body1.GetPosition();
	const JPH::Vec3 body_position2 = p_body2.GetPosition();

	for (const JPH::Vec3& relative_point : p_manifold.mRelativeContactPointsOn1) {
		Contact& contact = manifold.contacts1.emplace_back();

		const JPH::Vec3 world_normal = -p_manifold.mWorldSpaceNormal;
		const JPH::Vec3 world_point = p_manifold.mBaseOffset + relative_point;

		contact.normal = world_normal;
		contact.point_self = world_point - body_position1;
		contact.point_other = world_point - body_position2;
		contact.velocity_other = p_body2.GetPointVelocity(world_point);
		contact.impulse = calculate_contact_impulse(p_body1, p_body2, world_point, world_normal);
	}

	for (const JPH::Vec3& relative_point : p_manifold.mRelativeContactPointsOn2) {
		Contact& contact = manifold.contacts2.emplace_back();

		const JPH::Vec3 world_normal = p_manifold.mWorldSpaceNormal;
		const JPH::Vec3 world_point = p_manifold.mBaseOffset + relative_point;

		contact.normal = world_normal;
		contact.point_self = world_point - body_position2;
		contact.point_other = world_point - body_position1;
		contact.velocity_other = p_body1.GetPointVelocity(world_point);
		contact.impulse = calculate_contact_impulse(p_body2, p_body1, world_point, world_normal);
	}
}

void JoltContactListener::flush_contacts() {
	for (auto&& [shape_pair, manifold] : manifolds_by_shape_pair) {
		const JPH::BodyID body_ids[] = {shape_pair.GetBody1ID(), shape_pair.GetBody2ID()};

		const JoltReadableBodies3D jolt_bodies =
			space->read_bodies(body_ids, count_of(body_ids), false);

		auto* body1 = jolt_bodies[0].as<JoltBody3D>();
		ERR_FAIL_NULL(body1);

		auto* body2 = jolt_bodies[1].as<JoltBody3D>();
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
				contact.impulse
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
				contact.impulse
			);
		}

		manifold.contacts1.clear();
		manifold.contacts2.clear();
	}
}

void JoltContactListener::flush_area_shifts() {
	for (const JPH::SubShapeIDPair& shape_pair : area_overlaps) {
		auto is_shifted = [&](const JPH::BodyID& p_body_id, const JPH::SubShapeID& p_sub_shape_id) {
			const JoltReadableBody3D jolt_body = space->read_body(p_body_id, false);
			auto* object = jolt_body.as<JoltCollisionObject3D>();
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

		auto as_area = [](const JoltReadableBody3D& p_jolt_body) -> JoltArea3D* {
			if (p_jolt_body->IsSensor()) {
				return p_jolt_body.as<JoltArea3D>();
			} else {
				return nullptr;
			}
		};

		JoltArea3D* area1 = as_area(jolt_body1);
		JoltArea3D* area2 = as_area(jolt_body2);

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

		auto as_area = [](const JoltReadableBody3D& p_jolt_body) -> JoltArea3D* {
			if (p_jolt_body.is_valid() && p_jolt_body->IsSensor()) {
				return p_jolt_body.as<JoltArea3D>();
			} else {
				return nullptr;
			}
		};

		auto as_body = [](const JoltReadableBody3D& p_jolt_body) -> JoltBody3D* {
			if (p_jolt_body.is_valid() && !p_jolt_body->IsSensor()) {
				return p_jolt_body.as<JoltBody3D>();
			} else {
				return nullptr;
			}
		};

		JoltArea3D* area1 = as_area(jolt_body1);
		JoltArea3D* area2 = as_area(jolt_body2);

		JoltBody3D* body1 = as_body(jolt_body1);
		JoltBody3D* body2 = as_body(jolt_body2);

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
