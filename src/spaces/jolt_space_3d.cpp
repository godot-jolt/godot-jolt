#include "jolt_space_3d.hpp"

#include "joints/jolt_joint_3d.hpp"
#include "objects/jolt_area_3d.hpp"
#include "objects/jolt_body_3d.hpp"
#include "servers/jolt_project_settings.hpp"
#include "spaces/jolt_contact_listener_3d.hpp"
#include "spaces/jolt_layer_mapper.hpp"
#include "spaces/jolt_physics_direct_space_state_3d.hpp"
#include "spaces/jolt_temp_allocator.hpp"

namespace {

constexpr double GDJOLT_SPACE_CONTACT_RECYCLE_RADIUS = 0.01;
constexpr double GDJOLT_SPACE_CONTACT_MAX_SEPARATION = 0.05;
constexpr double GDJOLT_SPACE_CONTACT_MAX_ALLOWED_PENETRATION = 0.01;
constexpr double GDJOLT_SPACE_CONTACT_DEFAULT_BIAS = 0.8;
constexpr double GDJOLT_SPACE_SLEEP_THRESHOLD_LINEAR = 0.1;
constexpr double GDJOLT_SPACE_SLEEP_THRESHOLD_ANGULAR = 8.0 * Math_PI / 180;
constexpr double GDJOLT_SPACE_SOLVER_ITERATIONS = 8;

} // namespace

JoltSpace3D::JoltSpace3D(JPH::JobSystem* p_job_system)
	: job_system(p_job_system)
	, temp_allocator(new JoltTempAllocator())
	, layer_mapper(new JoltLayerMapper())
	, contact_listener(new JoltContactListener3D(this))
	, physics_system(new JPH::PhysicsSystem())
	, body_accessor(this) {
	physics_system->Init(
		(JPH::uint)JoltProjectSettings::get_max_bodies(),
		0,
		(JPH::uint)JoltProjectSettings::get_max_body_pairs(),
		(JPH::uint)JoltProjectSettings::get_max_contact_constraints(),
		*layer_mapper,
		*layer_mapper,
		*layer_mapper
	);

	JPH::PhysicsSettings settings;
	settings.mBaumgarte = JoltProjectSettings::get_stabilization_factor();
	settings.mSpeculativeContactDistance = JoltProjectSettings::get_contact_distance();
	settings.mPenetrationSlop = JoltProjectSettings::get_contact_penetration();
	settings.mNumVelocitySteps = JoltProjectSettings::get_velocity_iterations();
	settings.mNumPositionSteps = JoltProjectSettings::get_position_iterations();
	settings.mTimeBeforeSleep = JoltProjectSettings::get_sleep_time_threshold();
	settings.mPointVelocitySleepThreshold = JoltProjectSettings::get_sleep_velocity_threshold();
	settings.mDeterministicSimulation = JoltProjectSettings::is_more_deterministic();
	settings.mAllowSleeping = JoltProjectSettings::is_sleep_enabled();

	physics_system->SetPhysicsSettings(settings);
	physics_system->SetGravity(JPH::Vec3::sZero());
	physics_system->SetContactListener(contact_listener);
}

JoltSpace3D::~JoltSpace3D() {
	memdelete_safely(direct_state);
	delete_safely(physics_system);
	delete_safely(contact_listener);
	delete_safely(layer_mapper);
	delete_safely(temp_allocator);
}

void JoltSpace3D::step(float p_step) {
	last_step = p_step;

	pre_step(p_step);

	switch (physics_system->Update(p_step, 1, 1, temp_allocator, job_system)) {
		case JPH::EPhysicsUpdateError::None: {
			// All good!
		} break;

		case JPH::EPhysicsUpdateError::ManifoldCacheFull: {
			WARN_PRINT_ONCE(vformat(
				"Jolt's manifold cache exceeded capacity and contacts were ignored. "
				"Consider increasing maximum number of contact constraints in project settings. "
				"Maximum number of contact constraints is currently set to %d.",
				JoltProjectSettings::get_max_contact_constraints()
			));
		} break;

		case JPH::EPhysicsUpdateError::BodyPairCacheFull: {
			WARN_PRINT_ONCE(vformat(
				"Jolt's body pair cache exceeded capacity and contacts were ignored. "
				"Consider increasing maximum number of body pairs in project settings. "
				"Maximum number of body pairs is currently set to %d.",
				JoltProjectSettings::get_max_body_pairs()
			));
		} break;

		case JPH::EPhysicsUpdateError::ContactConstraintsFull: {
			WARN_PRINT_ONCE(vformat(
				"Jolt's contact constraint buffer exceeded capacity and contacts were ignored. "
				"Consider increasing maximum number of contact constraints in project settings. "
				"Maximum number of contact constraints is currently set to %d.",
				JoltProjectSettings::get_max_contact_constraints()
			));
		} break;
	}

	post_step(p_step);

	has_stepped = true;
}

void JoltSpace3D::call_queries() {
	if (!has_stepped) {
		// HACK(mihe): We need to skip the first invocation of this method, because there will be
		// pending notifications that need to be flushed first, which can cause weird conflicts with
		// things like `_integrate_forces`. This happens to also emulate the behavior of Godot
		// Physics, where (active) collision objects must register to have `call_queries` invoked,
		// which they don't do until the physics step, which happens after this.
		return;
	}

	body_accessor.acquire_active();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (!body->IsSensor() && !body->IsStatic()) {
				reinterpret_cast<JoltBody3D*>(body->GetUserData())->call_queries();
			}
		}
	}

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* body = body_accessor.try_get(i)) {
			if (body->IsSensor()) {
				reinterpret_cast<JoltArea3D*>(body->GetUserData())->call_queries();
			}
		}
	}

	body_accessor.release();
}

double JoltSpace3D::get_param(PhysicsServer3D::SpaceParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: {
			return GDJOLT_SPACE_CONTACT_RECYCLE_RADIUS;
		}
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: {
			return GDJOLT_SPACE_CONTACT_MAX_SEPARATION;
		}
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: {
			return GDJOLT_SPACE_CONTACT_MAX_ALLOWED_PENETRATION;
		}
		case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: {
			return GDJOLT_SPACE_CONTACT_DEFAULT_BIAS;
		}
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: {
			return GDJOLT_SPACE_SLEEP_THRESHOLD_LINEAR;
		}
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: {
			return GDJOLT_SPACE_SLEEP_THRESHOLD_ANGULAR;
		}
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: {
			return JoltProjectSettings::get_sleep_time_threshold();
		}
		case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: {
			return GDJOLT_SPACE_SOLVER_ITERATIONS;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled space parameter: '%d'", p_param));
		}
	}
}

void JoltSpace3D::set_param(
	PhysicsServer3D::SpaceParameter p_param,
	[[maybe_unused]] double p_value
) {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS: {
			WARN_PRINT(
				"Space-specific contact recycle radius is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION: {
			WARN_PRINT(
				"Space-specific contact max separation is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION: {
			WARN_PRINT(
				"Space-specific contact max allowed penetration is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_CONTACT_DEFAULT_BIAS: {
			WARN_PRINT(
				"Space-specific contact default bias is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD: {
			WARN_PRINT(
				"Space-specific linear velocity sleep threshold is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD: {
			WARN_PRINT(
				"Space-specific angular velocity sleep threshold is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP: {
			WARN_PRINT(
				"Space-specific body sleep time is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		case PhysicsServer3D::SPACE_PARAM_SOLVER_ITERATIONS: {
			WARN_PRINT(
				"Space-specific solver iterations is not supported by Godot Jolt. "
				"Any such value will be ignored."
			);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled space parameter: '%d'", p_param));
		} break;
	}
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

const JPH::BodyLockInterface& JoltSpace3D::get_lock_iface(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetBodyLockInterface();
	} else {
		return physics_system->GetBodyLockInterfaceNoLock();
	}
}

const JPH::BroadPhaseQuery& JoltSpace3D::get_broad_phase_query() const {
	return physics_system->GetBroadPhaseQuery();
}

const JPH::NarrowPhaseQuery& JoltSpace3D::get_narrow_phase_query(bool p_locked) const {
	if (p_locked && body_accessor.not_acquired()) {
		return physics_system->GetNarrowPhaseQuery();
	} else {
		return physics_system->GetNarrowPhaseQueryNoLock();
	}
}

JPH::ObjectLayer JoltSpace3D::map_to_object_layer(
	JPH::BroadPhaseLayer p_broad_phase_layer,
	uint32_t p_collision_layer,
	uint32_t p_collision_mask
) {
	return layer_mapper->to_object_layer(p_broad_phase_layer, p_collision_layer, p_collision_mask);
}

void JoltSpace3D::map_from_object_layer(
	JPH::ObjectLayer p_object_layer,
	JPH::BroadPhaseLayer& p_broad_phase_layer,
	uint32_t& p_collision_layer,
	uint32_t& p_collision_mask
) const {
	layer_mapper->from_object_layer(
		p_object_layer,
		p_broad_phase_layer,
		p_collision_layer,
		p_collision_mask
	);
}

JoltReadableBody3D JoltSpace3D::read_body(const JPH::BodyID& p_body_id, bool p_lock) const {
	return {*this, p_body_id, p_lock};
}

JoltReadableBody3D JoltSpace3D::read_body(const JoltCollisionObject3D& p_object, bool p_lock)
	const {
	return read_body(p_object.get_jolt_id(), p_lock);
}

JoltWritableBody3D JoltSpace3D::write_body(const JPH::BodyID& p_body_id, bool p_lock) const {
	return {*this, p_body_id, p_lock};
}

JoltWritableBody3D JoltSpace3D::write_body(const JoltCollisionObject3D& p_object, bool p_lock)
	const {
	return write_body(p_object.get_jolt_id(), p_lock);
}

JoltReadableBodies3D JoltSpace3D::read_bodies(
	const JPH::BodyID* p_body_ids,
	int32_t p_body_count,
	bool p_lock
) const {
	return {*this, p_body_ids, p_body_count, p_lock};
}

JoltWritableBodies3D JoltSpace3D::write_bodies(
	const JPH::BodyID* p_body_ids,
	int32_t p_body_count,
	bool p_lock
) const {
	return {*this, p_body_ids, p_body_count, p_lock};
}

JoltPhysicsDirectSpaceState3D* JoltSpace3D::get_direct_state() {
	if (direct_state == nullptr) {
		direct_state = memnew(JoltPhysicsDirectSpaceState3D(this));
	}

	return direct_state;
}

void JoltSpace3D::add_joint(JPH::Constraint* p_jolt_ref) {
	physics_system->AddConstraint(p_jolt_ref);
}

void JoltSpace3D::add_joint(JoltJoint3D* p_joint) {
	add_joint(p_joint->get_jolt_ref());
}

void JoltSpace3D::remove_joint(JPH::Constraint* p_jolt_ref) {
	physics_system->RemoveConstraint(p_jolt_ref);
}

void JoltSpace3D::remove_joint(JoltJoint3D* p_joint) {
	remove_joint(p_joint->get_jolt_ref());
}

#ifdef DEBUG_ENABLED

const PackedVector3Array& JoltSpace3D::get_debug_contacts() const {
	return contact_listener->get_debug_contacts();
}

int32_t JoltSpace3D::get_debug_contact_count() const {
	return contact_listener->get_debug_contact_count();
}

int32_t JoltSpace3D::get_max_debug_contacts() const {
	return contact_listener->get_max_debug_contacts();
}

void JoltSpace3D::set_max_debug_contacts(int32_t p_count) {
	contact_listener->set_max_debug_contacts(p_count);
}

#endif // DEBUG_ENABLED

void JoltSpace3D::pre_step(float p_step) {
	body_accessor.acquire_all(true);

	contact_listener->pre_step();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* jolt_body = body_accessor.try_get(i)) {
			auto* object = reinterpret_cast<JoltCollisionObject3D*>(jolt_body->GetUserData());

			object->pre_step(p_step);

			if (object->generates_contacts()) {
				contact_listener->listen_for(object);
			}
		}
	}

	body_accessor.release();
}

void JoltSpace3D::post_step(float p_step) {
	body_accessor.acquire_all(true);

	contact_listener->post_step();

	const int32_t body_count = body_accessor.get_count();

	for (int32_t i = 0; i < body_count; ++i) {
		if (const JPH::Body* jolt_body = body_accessor.try_get(i)) {
			auto* object = reinterpret_cast<JoltCollisionObject3D*>(jolt_body->GetUserData());

			object->post_step(p_step);
		}
	}

	body_accessor.release();
}
