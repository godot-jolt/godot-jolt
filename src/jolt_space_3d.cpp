#include "jolt_space_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "jolt_broad_phase_layer.hpp"
#include "jolt_collision_object_3d.hpp"
#include "jolt_layer_mapper.hpp"
#include "jolt_object_layer.hpp"
#include "jolt_shape_3d.hpp"
#include "jolt_temp_allocator.hpp"

namespace {

constexpr uint32_t GDJOLT_TEMP_CAPACITY = 8 * 1024 * 1024;
constexpr uint32_t GDJOLT_MAX_BODIES = 8192;
constexpr uint32_t GDJOLT_NUM_BODY_MUTEXES = 0; // 0 = default
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
		GDJOLT_NUM_BODY_MUTEXES,
		GDJOLT_MAX_BODY_PAIRS,
		GDJOLT_MAX_CONTACT_CONSTRAINTS,
		*layer_mapper,
		&JoltLayerMapper::can_layers_collide,
		&JoltLayerMapper::can_layers_collide
	);
}

JoltSpace3D::~JoltSpace3D() {
	delete physics_system;
	delete layer_mapper;
	delete temp_allocator;
}

void JoltSpace3D::step(float p_step) {
	lock();

	// TODO(mihe): Integrate forces/velocities

	physics_system->Update(p_step, 1, 1, temp_allocator, job_system);

	unlock();
}

void JoltSpace3D::call_queries() {
	JPH::BodyIDVector body_ids;
	physics_system->GetActiveBodies(body_ids);

	{
		const JPH::BodyLockInterface& lock_iface = physics_system->GetBodyLockInterface();
		const JPH::BodyLockMultiRead lock(lock_iface, body_ids.data(), (int)body_ids.size());

		// TODO(mihe): Is the separation of bodies and areas here important? Maybe merge into a
		// single loop?

		for (int i = 0; i < (int)body_ids.size(); ++i) {
			if (const JPH::Body* body = lock.GetBody(i)) {
				if (!body->IsSensor()) {
					auto* object = reinterpret_cast<JoltCollisionObject3D*>(body->GetUserData());
					object->call_queries();
				}
			}
		}

		for (int i = 0; i < (int)body_ids.size(); ++i) {
			if (const JPH::Body* body = lock.GetBody(i)) {
				if (body->IsSensor()) {
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

PhysicsDirectSpaceState3D* JoltSpace3D::get_direct_state() const {
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
	if (!p_object->get_jid().IsInvalid()) {
		return;
	}

	JPH::Ref compound_shape = new JPH::MutableCompoundShapeSettings();

	for (const JoltShapeInstance3D& shape : p_object->get_shapes()) {
		compound_shape->AddShape(
			to_jolt(shape.get_transform().origin),
			to_jolt(shape.get_transform().basis),
			shape->get_jref()
		);
	}

	JPH::EMotionType motion_type = {};
	JPH::ObjectLayer object_layer = {};

	switch (p_object->get_mode()) {
		case PhysicsServer3D::BODY_MODE_STATIC: {
			motion_type = JPH::EMotionType::Static;
			object_layer = GDJOLT_OBJECT_LAYER_STATIC;
		} break;
		case PhysicsServer3D::BODY_MODE_KINEMATIC: {
			motion_type = JPH::EMotionType::Kinematic;
			object_layer = GDJOLT_OBJECT_LAYER_MOVING;
		} break;
		case PhysicsServer3D::BODY_MODE_RIGID:
		case PhysicsServer3D::BODY_MODE_RIGID_LINEAR: {
			motion_type = JPH::EMotionType::Dynamic;
			object_layer = GDJOLT_OBJECT_LAYER_MOVING;
		} break;
		default: {
			ERR_FAIL_MSG("Unhandled body mode");
		} break;
	}

	const Transform3D& transform = p_object->get_initial_transform();
	const bool is_sensor = p_object->is_sensor();

	JPH::BodyCreationSettings settings(
		compound_shape,
		to_jolt(transform.origin),
		to_jolt(transform.basis),
		motion_type,
		object_layer
	);

	settings.mLinearVelocity = to_jolt(p_object->get_initial_linear_velocity());
	settings.mAngularVelocity = to_jolt(p_object->get_initial_angular_velocity());
	settings.mAllowDynamicOrKinematic = true;
	settings.mIsSensor = is_sensor;
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

	p_object->set_jid(body->GetID());
}

void JoltSpace3D::add_object(JoltCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().AddBody(
		p_object->get_jid(),
		p_object->get_initial_sleep_state() ? JPH::EActivation::DontActivate
											: JPH::EActivation::Activate
	);
}

void JoltSpace3D::remove_object(JoltCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().RemoveBody(p_object->get_jid());
}

void JoltSpace3D::destroy_object(JoltCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().DestroyBody(p_object->get_jid());
	p_object->set_jid(JPH::BodyID());
}

void JoltSpace3D::update_gravity() {
	physics_system->SetGravity(to_jolt(gravity_vector * gravity));
}
