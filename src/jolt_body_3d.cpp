#include "jolt_body_3d.hpp"

#include "jolt_body_access_3d.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"
#include "jolt_space_3d.hpp"

JoltBody3D::~JoltBody3D() {
	memdelete_safely(direct_state);
}

Variant JoltBody3D::get_state(PhysicsServer3D::BodyState p_state) {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM: {
			return get_transform();
		}
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY: {
			return get_linear_velocity();
		}
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY: {
			return get_angular_velocity();
		}
		case PhysicsServer3D::BODY_STATE_SLEEPING: {
			return get_sleep_state();
		}
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP: {
			return can_sleep();
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled body state: '{}'", p_state));
		}
	}
}

void JoltBody3D::set_state(PhysicsServer3D::BodyState p_state, const Variant& p_value) {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM: {
			set_transform(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY: {
			set_linear_velocity(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY: {
			set_angular_velocity(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_SLEEPING: {
			set_sleep_state(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP: {
			set_can_sleep(p_value);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled body state: '{}'", p_state));
		} break;
	}
}

Variant JoltBody3D::get_param(PhysicsServer3D::BodyParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_BOUNCE: {
			return get_bounce();
		}
		case PhysicsServer3D::BODY_PARAM_FRICTION: {
			return get_friction();
		}
		case PhysicsServer3D::BODY_PARAM_MASS: {
			return get_mass();
		}
		case PhysicsServer3D::BODY_PARAM_INERTIA: {
			return get_inertia();
		}
		case PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS: {
			return get_center_of_mass_custom();
		}
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE: {
			return get_gravity_scale();
		}
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP_MODE: {
			ERR_FAIL_D_NOT_IMPL();
		}
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP_MODE: {
			ERR_FAIL_D_NOT_IMPL();
		}
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			return get_linear_damp();
		}
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: {
			return get_angular_damp();
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled body parameter: '{}'", p_param));
		}
	}
}

void JoltBody3D::set_param(PhysicsServer3D::BodyParameter p_param, const Variant& p_value) {
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_BOUNCE: {
			set_bounce(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_FRICTION: {
			set_friction(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_MASS: {
			set_mass(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_INERTIA: {
			set_inertia(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS: {
			set_center_of_mass_custom(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE: {
			set_gravity_scale(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP_MODE: {
			ERR_FAIL_NOT_IMPL();
		} break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP_MODE: {
			ERR_FAIL_NOT_IMPL();
		} break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			set_linear_damp(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: {
			set_angular_damp(p_value);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled body parameter: '{}'", p_param));
		} break;
	}
}

void JoltBody3D::set_state_sync_callback(const Callable& p_callback) {
	body_state_callback = p_callback;
}

bool JoltBody3D::get_sleep_state(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return !body_access.get_body().IsActive();
}

void JoltBody3D::set_sleep_state(bool p_enabled, bool p_lock) {
	if (!space) {
		initial_sleep_state = p_enabled;
		return;
	}

	if (p_enabled) {
		space->get_body_iface(p_lock).DeactivateBody(jolt_id);
	} else {
		space->get_body_iface(p_lock).ActivateBody(jolt_id);
	}
}

void JoltBody3D::set_can_sleep(bool p_enabled, bool p_lock) {
	if (p_enabled == allowed_sleep) {
		return;
	}

	allowed_sleep = p_enabled;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetAllowSleeping(allowed_sleep);
}

Basis JoltBody3D::get_inverse_inertia_tensor(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	if (mode != PhysicsServer3D::BodyMode::BODY_MODE_RIGID) {
		return {};
	}

	const JoltBodyAccessRead3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetInverseInertia().GetQuaternion());
}

Vector3 JoltBody3D::get_linear_velocity(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetMotionPropertiesUnchecked()->GetLinearVelocity());
}

void JoltBody3D::set_linear_velocity(const Vector3& p_velocity, bool p_lock) {
	if (!space) {
		initial_linear_velocity = p_velocity;
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().GetMotionPropertiesUnchecked()->SetLinearVelocityClamped(
		to_jolt(p_velocity)
	);
}

Vector3 JoltBody3D::get_angular_velocity(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetMotionPropertiesUnchecked()->GetAngularVelocity());
}

void JoltBody3D::set_angular_velocity(const Vector3& p_velocity, bool p_lock) {
	if (!space) {
		initial_angular_velocity = p_velocity;
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().GetMotionPropertiesUnchecked()->SetAngularVelocityClamped(
		to_jolt(p_velocity)
	);
}

void JoltBody3D::set_center_of_mass_custom(const Vector3& p_center_of_mass, bool p_lock) {
	custom_center_of_mass = true;
	center_of_mass_custom = p_center_of_mass;
	rebuild_shape(p_lock);
}

void JoltBody3D::reset_mass_properties(bool p_lock) {
	inertia.zero();
	custom_center_of_mass = false;
	center_of_mass_custom.zero();
	mass_properties_changed(p_lock);
}

void JoltBody3D::call_queries() {
	// TODO(mihe): Call force integration callback

	if (body_state_callback.is_valid()) {
		body_state_callback.callv(Array::make(get_direct_state()));
	}
}

JoltPhysicsDirectBodyState3D* JoltBody3D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(JoltPhysicsDirectBodyState3D(this));
	}

	return direct_state;
}

void JoltBody3D::set_mode(PhysicsServer3D::BodyMode p_mode, bool p_lock) {
	if (p_mode == PhysicsServer3D::BODY_MODE_RIGID_LINEAR) {
		WARN_PRINT(
			"Locking rotation is not supported by Godot Jolt. "
			"Any such setting will be treated as disabled."
		);

		p_mode = PhysicsServer3D::BODY_MODE_RIGID;
	}

	if (p_mode == mode) {
		return;
	}

	mode = p_mode;

	if (!space) {
		return;
	}

	JPH::EMotionType motion_type = {};

	switch (p_mode) {
		case PhysicsServer3D::BODY_MODE_STATIC: {
			motion_type = JPH::EMotionType::Static;
		} break;
		case PhysicsServer3D::BODY_MODE_KINEMATIC: {
			motion_type = JPH::EMotionType::Kinematic;
		} break;
		case PhysicsServer3D::BODY_MODE_RIGID: {
			motion_type = JPH::EMotionType::Dynamic;
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled body mode: '{}'", p_mode));
		} break;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	if (!get_sleep_state(false) && motion_type == JPH::EMotionType::Static) {
		set_sleep_state(true, false);
	}

	JPH::Body& body = body_access.get_body();

	body.SetMotionType(motion_type);

	if (get_sleep_state(false) && motion_type != JPH::EMotionType::Static) {
		set_sleep_state(false, false);
	}

	if (motion_type == JPH::EMotionType::Kinematic) {
		body.SetLinearVelocity(JPH::Vec3::sZero());
		body.SetAngularVelocity(JPH::Vec3::sZero());
	}
}

void JoltBody3D::set_ccd_enabled(bool p_enable, bool p_lock) {
	if (p_enable == ccd_enabled) {
		return;
	}

	ccd_enabled = p_enable;

	if (!space) {
		return;
	}

	space->get_body_iface(p_lock).SetMotionQuality(
		jolt_id,
		ccd_enabled ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete
	);
}

void JoltBody3D::set_mass(float p_mass, bool p_lock) {
	if (p_mass != mass) {
		mass = p_mass;
		mass_properties_changed(p_lock);
	}
}

void JoltBody3D::set_inertia(const Vector3& p_inertia, bool p_lock) {
	if (p_inertia != inertia) {
		inertia = p_inertia;
		mass_properties_changed(p_lock);
	}
}

void JoltBody3D::set_bounce(float p_bounce, bool p_lock) {
	if (p_bounce < 0.0f || p_bounce > 1.0f) {
		WARN_PRINT(
			"Bounce values less than 0 or greater than 1 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);

		p_bounce = clamp(p_bounce, 0.0f, 1.0f);
	}

	if (p_bounce == bounce) {
		return;
	}

	bounce = p_bounce;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetRestitution(bounce);
}

void JoltBody3D::set_friction(float p_friction, bool p_lock) {
	if (p_friction < 0.0f) {
		WARN_PRINT(
			"Friction values less than 0 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);

		p_friction = 0;
	}

	if (p_friction == friction) {
		return;
	}

	friction = p_friction;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().SetFriction(friction);
}

void JoltBody3D::set_gravity_scale(float p_scale, bool p_lock) {
	if (p_scale == gravity_scale) {
		return;
	}

	gravity_scale = p_scale;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().GetMotionPropertiesUnchecked()->SetGravityFactor(p_scale);
}

void JoltBody3D::set_linear_damp(float p_damp, bool p_lock) {
	if (p_damp < 0.0f) {
		WARN_PRINT(
			"Linear damp values less than 0 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);

		p_damp = 0;
	}

	if (p_damp == linear_damp) {
		return;
	}

	linear_damp = p_damp;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().GetMotionPropertiesUnchecked()->SetLinearDamping(linear_damp);
}

void JoltBody3D::set_angular_damp(float p_damp, bool p_lock) {
	if (p_damp < 0.0f) {
		WARN_PRINT(
			"Angular damp values less than 0 are not supported by Godot Jolt. "
			"Values outside this range will be clamped."
		);

		p_damp = 0;
	}

	if (p_damp == angular_damp) {
		return;
	}

	angular_damp = p_damp;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	body_access.get_body().GetMotionPropertiesUnchecked()->SetAngularDamping(angular_damp);
}

void JoltBody3D::shapes_changed(bool p_lock) {
	mass_properties_changed(p_lock);
}

void JoltBody3D::mass_properties_changed(bool p_lock) {
	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	const JPH::MassProperties mass_properties = calculate_mass_properties(false);
	body_access.get_body().GetMotionPropertiesUnchecked()->SetMassProperties(mass_properties);
}
