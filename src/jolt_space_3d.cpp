#include "jolt_space_3d.hpp"

#include "jolt_body_3d.hpp"
#include "jolt_body_access_3d.hpp"
#include "jolt_broad_phase_layer.hpp"
#include "jolt_collision_object_3d.hpp"
#include "jolt_joint_3d.hpp"
#include "jolt_layer_mapper.hpp"
#include "jolt_object_layer.hpp"
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

JoltSpace3D::JoltSpace3D(JPH::JobSystem* p_job_system, JPH::GroupFilter* p_group_filter)
	: job_system(p_job_system)
	, temp_allocator(new JoltTempAllocator(GDJOLT_TEMP_CAPACITY))
	, layer_mapper(new JoltLayerMapper())
	, physics_system(new JPH::PhysicsSystem())
	, group_filter(p_group_filter) {
	physics_system->Init(
		GDJOLT_MAX_BODIES,
		GDJOLT_BODY_MUTEX_COUNT,
		GDJOLT_MAX_BODY_PAIRS,
		GDJOLT_MAX_CONTACT_CONSTRAINTS,
		*layer_mapper,
		&JoltLayerMapper::can_layers_collide,
		&JoltLayerMapper::can_layers_collide
	);
}

JoltSpace3D::~JoltSpace3D() {
	memdelete_safely(direct_state);
	delete_safely(physics_system);
	delete_safely(layer_mapper);
	delete_safely(temp_allocator);
}

void JoltSpace3D::step(float p_step) {
	locked = true;

	integrate_forces();

	physics_system->Update(p_step, 1, 1, temp_allocator, job_system);

	locked = false;
}

void JoltSpace3D::call_queries() {
	JPH::BodyIDVector body_ids;
	physics_system->GetActiveBodies(body_ids);

	{
		const JPH::BodyLockInterface& lock_iface = physics_system->GetBodyLockInterface();
		const JPH::BodyLockMultiRead lock(lock_iface, body_ids.data(), (int32_t)body_ids.size());

		// TODO(mihe): Is the separation of bodies and areas here important? Maybe merge into a
		// single loop?

		for (int32_t i = 0; i < (int32_t)body_ids.size(); ++i) {
			if (const JPH::Body* body = lock.GetBody(i)) {
				if (!body->IsStatic() && !body->IsSensor()) {
					auto* object = reinterpret_cast<JoltCollisionObject3D*>(body->GetUserData());
					object->call_queries();
				}
			}
		}

		for (int32_t i = 0; i < (int32_t)body_ids.size(); ++i) {
			if (const JPH::Body* body = lock.GetBody(i)) {
				if (!body->IsStatic() && body->IsSensor()) {
					auto* object = reinterpret_cast<JoltCollisionObject3D*>(body->GetUserData());
					object->call_queries();
				}
			}
		}
	}
}

JPH::BodyInterface& JoltSpace3D::get_body_iface(bool p_locked) {
	return p_locked ? physics_system->GetBodyInterface() : physics_system->GetBodyInterfaceNoLock();
}

const JPH::BodyInterface& JoltSpace3D::get_body_iface(bool p_locked) const {
	return p_locked ? physics_system->GetBodyInterface() : physics_system->GetBodyInterfaceNoLock();
}

const JPH::BodyLockInterface& JoltSpace3D::get_body_lock_iface(bool p_locked) const {
	if (p_locked) {
		return physics_system->GetBodyLockInterface();
	} else {
		return physics_system->GetBodyLockInterfaceNoLock();
	}
}

const JPH::NarrowPhaseQuery& JoltSpace3D::get_narrow_phase_query(bool p_locked) const {
	if (p_locked) {
		return physics_system->GetNarrowPhaseQuery();
	} else {
		return physics_system->GetNarrowPhaseQueryNoLock();
	}
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
			update_gravity();
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			gravity_vector = p_value;
			update_gravity();
		} break;
		default: {
			ERR_FAIL_NOT_IMPL();
		} break;
	}
}

void JoltSpace3D::create_object(JoltCollisionObject3D* p_object) {
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
			ERR_FAIL_MSG(vformat("Unhandled body mode: '{}'", body_mode));
		} break;
	}

	JPH::ShapeRefC shape = p_object->try_build_shape();
	JPH::ObjectLayer object_layer = JoltLayerMapper::to_object_layer(motion_type);

	if (shape == nullptr) {
		shape = new JPH::SphereShape(1.0f);
		object_layer = GDJOLT_OBJECT_LAYER_NONE;
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

	JPH::Body* body = physics_system->GetBodyInterface().CreateBody(settings);

	body->SetCollisionGroup(JPH::CollisionGroup(
		group_filter,
		p_object->get_collision_layer(),
		p_object->get_collision_mask()
	));

	body->SetUserData(reinterpret_cast<JPH::uint64>(p_object));

	p_object->set_jolt_id(body->GetID());
}

void JoltSpace3D::add_object(JoltCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().AddBody(
		p_object->get_jolt_id(),
		p_object->get_initial_sleep_state() ? JPH::EActivation::DontActivate
											: JPH::EActivation::Activate
	);
}

void JoltSpace3D::remove_object(JoltCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().RemoveBody(p_object->get_jolt_id());
}

void JoltSpace3D::destroy_object(JoltCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().DestroyBody(p_object->get_jolt_id());
	p_object->set_jolt_id({});
}

void JoltSpace3D::add_joint(JoltJoint3D* p_joint) {
	physics_system->AddConstraint(p_joint->get_jolt_ref());
}

void JoltSpace3D::remove_joint(JoltJoint3D* p_joint) {
	physics_system->RemoveConstraint(p_joint->get_jolt_ref());
}

void JoltSpace3D::integrate_forces(bool p_lock) {
	JPH::BodyIDVector body_ids;
	physics_system->GetBodies(body_ids);

	const auto body_count = (int32_t)body_ids.size();
	JoltMultiBodyAccessWrite3D body_access(*this, body_ids.data(), body_count, p_lock);

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_access.get_body(i)) {
			if (!body->IsStatic() && !body->IsSensor()) {
				reinterpret_cast<JoltBody3D*>(body->GetUserData())->integrate_forces(false);
			}
		}
	}
}

void JoltSpace3D::update_gravity() {
	physics_system->SetGravity(to_jolt(gravity_vector * gravity));
}
