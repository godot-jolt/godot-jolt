#include "jolt_space_3d.hpp"

#include "jolt_area_3d.hpp"
#include "jolt_body_3d.hpp"
#include "jolt_broad_phase_layer.hpp"
#include "jolt_collision_object_3d.hpp"
#include "jolt_group_filter_rid.hpp"
#include "jolt_joint_3d.hpp"
#include "jolt_layer_mapper.hpp"
#include "jolt_physics_direct_space_state_3d.hpp"
#include "jolt_shape_3d.hpp"
#include "jolt_temp_allocator.hpp"

namespace {

constexpr uint32_t GDJOLT_TEMP_CAPACITY = 8 * 1024 * 1024;
constexpr uint32_t GDJOLT_MAX_BODIES = 8192;
constexpr uint32_t GDJOLT_BODY_MUTEX_COUNT = 0; // 0 = default
constexpr uint32_t GDJOLT_MAX_BODY_PAIRS = 65536;
constexpr uint32_t GDJOLT_MAX_CONTACT_CONSTRAINTS = 8192;

} // namespace

JoltSpace3D::JoltSpace3D(JPH::JobSystem* p_job_system)
	: job_system(p_job_system)
	, temp_allocator(new JoltTempAllocator(GDJOLT_TEMP_CAPACITY))
	, layer_mapper(new JoltLayerMapper())
	, physics_system(new JPH::PhysicsSystem())
	, body_accessor(this) {
	physics_system->Init(
		GDJOLT_MAX_BODIES,
		GDJOLT_BODY_MUTEX_COUNT,
		GDJOLT_MAX_BODY_PAIRS,
		GDJOLT_MAX_CONTACT_CONSTRAINTS,
		*layer_mapper,
		*layer_mapper,
		*layer_mapper
	);
}

JoltSpace3D::~JoltSpace3D() {
	memdelete_safely(direct_state);
	delete_safely(physics_system);
	delete_safely(layer_mapper);
	delete_safely(temp_allocator);
}

void JoltSpace3D::step(float p_step) {
	integrate_forces();

	physics_system->Update(p_step, 1, 1, temp_allocator, job_system);
}

void JoltSpace3D::call_queries() {
	body_accessor.acquire_active();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (!body->IsStatic() && !body->IsSensor()) {
				reinterpret_cast<JoltBody3D*>(body->GetUserData())->call_queries();
			}
		}
	}

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (!body->IsStatic() && body->IsSensor()) {
				reinterpret_cast<JoltArea3D*>(body->GetUserData())->call_queries();
			}
		}
	}

	body_accessor.release();
}

JPH::BodyInterface& JoltSpace3D::get_body_iface(bool p_locked) {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyInterface();
	} else {
		return physics_system->GetBodyInterfaceNoLock();
	}
}

const JPH::BodyInterface& JoltSpace3D::get_body_iface(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyInterface();
	} else {
		return physics_system->GetBodyInterfaceNoLock();
	}
}

const JPH::BodyLockInterface& JoltSpace3D::get_body_lock_iface(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyLockInterface();
	} else {
		return physics_system->GetBodyLockInterfaceNoLock();
	}
}

const JPH::NarrowPhaseQuery& JoltSpace3D::get_narrow_phase_query(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetNarrowPhaseQuery();
	} else {
		return physics_system->GetNarrowPhaseQueryNoLock();
	}
}

JPH::ObjectLayer JoltSpace3D::map_to_object_layer(
	JPH::EMotionType p_motion_type,
	uint32_t p_collision_layer,
	uint32_t p_collision_mask
) {
	return layer_mapper->to_object_layer(p_motion_type, p_collision_layer, p_collision_mask);
}

JoltReadableBody3D JoltSpace3D::read_body(const JPH::BodyID& p_body_id, bool p_lock) const {
	return {*this, p_body_id, p_lock};
}

JoltReadableBody3D JoltSpace3D::read_body(const JoltBody3D& p_body, bool p_lock) const {
	return read_body(p_body.get_jolt_id(), p_lock);
}

JoltWritableBody3D JoltSpace3D::write_body(const JPH::BodyID& p_body_id, bool p_lock) const {
	return {*this, p_body_id, p_lock};
}

JoltWritableBody3D JoltSpace3D::write_body(const JoltBody3D& p_body, bool p_lock) const {
	return write_body(p_body.get_jolt_id(), p_lock);
}

JoltReadableBodies3D JoltSpace3D::read_bodies(
	const JPH::BodyID* p_body_ids,
	int p_body_count,
	bool p_lock
) const {
	return {*this, p_body_ids, p_body_count, p_lock};
}

JoltWritableBodies3D JoltSpace3D::write_bodies(
	const JPH::BodyID* p_body_ids,
	int p_body_count,
	bool p_lock
) const {
	return {*this, p_body_ids, p_body_count, p_lock};
}

JoltPhysicsDirectSpaceState3D* JoltSpace3D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(JoltPhysicsDirectSpaceState3D(this));
	}

	return direct_state;
}

Variant JoltSpace3D::get_param(PhysicsServer3D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			return gravity;
		}
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			return gravity_vector;
		}
		default: {
			ERR_FAIL_D_NOT_IMPL();
		}
	}
}

void JoltSpace3D::set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			gravity = p_value;
			gravity_changed();
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			gravity_vector = p_value;
			gravity_changed();
		} break;
		default: {
			ERR_FAIL_NOT_IMPL();
		} break;
	}
}

void JoltSpace3D::create_object(JoltCollisionObject3D* p_object, bool p_lock) {
	const PhysicsServer3D::BodyMode body_mode = p_object->get_mode();

	JPH::EMotionType motion_type = {};

	switch (body_mode) {
		case PhysicsServer3D::BODY_MODE_STATIC: {
			motion_type = JPH::EMotionType::Static;
		} break;
		case PhysicsServer3D::BODY_MODE_KINEMATIC: {
			motion_type = JPH::EMotionType::Kinematic;
		} break;
		case PhysicsServer3D::BODY_MODE_RIGID:
		case PhysicsServer3D::BODY_MODE_RIGID_LINEAR: {
			motion_type = JPH::EMotionType::Dynamic;
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled body mode: '%d'", body_mode));
		} break;
	}

	JPH::ShapeRefC shape = p_object->try_build_shape();
	JPH::ObjectLayer object_layer = map_to_object_layer(
		motion_type,
		p_object->get_collision_layer(),
		p_object->get_collision_mask()
	);

	if (shape == nullptr) {
		shape = new JPH::SphereShape(1.0f);
		object_layer = 0;
	}

	const Transform3D& transform = p_object->get_initial_transform();

	JPH::BodyCreationSettings settings(
		shape,
		to_jolt(transform.origin),
		to_jolt(transform.basis),
		motion_type,
		object_layer
	);

	settings.mLinearVelocity = to_jolt(p_object->get_initial_linear_velocity());
	settings.mAngularVelocity = to_jolt(p_object->get_initial_angular_velocity());
	settings.mAllowDynamicOrKinematic = true;
	settings.mIsSensor = p_object->is_area();
	settings.mMotionQuality = p_object->is_ccd_enabled() ? JPH::EMotionQuality::LinearCast
														 : JPH::EMotionQuality::Discrete;
	settings.mAllowSleeping = p_object->can_sleep();
	settings.mFriction = p_object->get_friction();
	settings.mRestitution = p_object->get_bounce();
	settings.mLinearDamping = p_object->get_linear_damp();
	settings.mAngularDamping = p_object->get_angular_damp();
	settings.mGravityFactor = p_object->get_gravity_scale();
	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
	settings.mMassPropertiesOverride = p_object->calculate_mass_properties(*settings.GetShape());

	JPH::Body* body = get_body_iface(p_lock).CreateBody(settings);

	body->SetUserData(reinterpret_cast<JPH::uint64>(p_object));

	if (!p_object->is_area()) {
		// HACK(mihe): Since group filters don't grant us access to user data we are instead forced
		// abuse the collision group to carry the upper and lower bits of our RID, which we can then
		// access and rebuild in our group filter for bodies that make use of collision exceptions.

		JPH::CollisionGroup::GroupID group_id = 0;
		JPH::CollisionGroup::SubGroupID sub_group_id = 0;
		JoltGroupFilterRID::encode_rid(p_object->get_rid(), group_id, sub_group_id);

		body->SetCollisionGroup(JPH::CollisionGroup(nullptr, group_id, sub_group_id));
	}

	p_object->set_jolt_id(body->GetID());
}

void JoltSpace3D::add_object(JoltCollisionObject3D* p_object, bool p_lock) {
	get_body_iface(p_lock).AddBody(
		p_object->get_jolt_id(),
		p_object->get_initial_sleep_state() ? JPH::EActivation::DontActivate
											: JPH::EActivation::Activate
	);
}

void JoltSpace3D::remove_object(JoltCollisionObject3D* p_object, bool p_lock) {
	get_body_iface(p_lock).RemoveBody(p_object->get_jolt_id());
}

void JoltSpace3D::destroy_object(JoltCollisionObject3D* p_object, bool p_lock) {
	get_body_iface(p_lock).DestroyBody(p_object->get_jolt_id());
	p_object->set_jolt_id({});
}

void JoltSpace3D::add_joint(JoltJoint3D* p_joint) {
	physics_system->AddConstraint(p_joint->get_jolt_ref());
}

void JoltSpace3D::remove_joint(JoltJoint3D* p_joint) {
	physics_system->RemoveConstraint(p_joint->get_jolt_ref());
}

void JoltSpace3D::integrate_forces(bool p_lock) {
	body_accessor.acquire_all(p_lock);

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (!body->IsStatic() && !body->IsSensor()) {
				reinterpret_cast<JoltBody3D*>(body->GetUserData())->integrate_forces(false);
			}
		}
	}

	body_accessor.release();
}

void JoltSpace3D::gravity_changed() {
	physics_system->SetGravity(to_jolt(gravity_vector * gravity));
}
