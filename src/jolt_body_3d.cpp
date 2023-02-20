#include "jolt_body_3d.hpp"

#include "jolt_area_3d.hpp"
#include "jolt_broad_phase_layer.hpp"
#include "jolt_group_filter_rid.hpp"
#include "jolt_physics_direct_body_state_3d.hpp"
#include "jolt_space_3d.hpp"

namespace {

template<typename TValue, typename TGetter>
bool integrate(TValue& p_value, PhysicsServer3D::AreaSpaceOverrideMode p_mode, TGetter&& p_getter) {
	switch (p_mode) {
		case PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED: {
			return false;
		}
		case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE: {
			p_value += p_getter();
			return false;
		}
		case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
			p_value += p_getter();
			return true;
		}
		case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE: {
			p_value = p_getter();
			return true;
		}
		case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
			p_value = p_getter();
			return false;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled override mode: '%d'", p_mode));
		}
	}
}

} // namespace

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
			ERR_FAIL_D_MSG(vformat("Unhandled body state: '%d'", p_state));
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
			ERR_FAIL_MSG(vformat("Unhandled body state: '%d'", p_state));
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
			return get_linear_damp_mode();
		}
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP_MODE: {
			return get_angular_damp_mode();
		}
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			return get_linear_damp();
		}
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: {
			return get_angular_damp();
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled body parameter: '%d'", p_param));
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
			set_linear_damp_mode((DampMode)(int32_t)p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP_MODE: {
			set_angular_damp_mode((DampMode)(int32_t)p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			set_linear_damp(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: {
			set_angular_damp(p_value);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled body parameter: '%d'", p_param));
		} break;
	}
}

JPH::BroadPhaseLayer JoltBody3D::get_broad_phase_layer() const {
	switch (mode) {
		case PhysicsServer3D::BODY_MODE_STATIC: {
			return JoltBroadPhaseLayer::BODY_STATIC;
		}
		case PhysicsServer3D::BODY_MODE_KINEMATIC:
		case PhysicsServer3D::BODY_MODE_RIGID:
		case PhysicsServer3D::BODY_MODE_RIGID_LINEAR: {
			return JoltBroadPhaseLayer::BODY_DYNAMIC;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled body mode: '%d'", mode));
		}
	}
}

bool JoltBody3D::get_sleep_state(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return !body->IsActive();
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

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->SetAllowSleeping(allowed_sleep);
}

Basis JoltBody3D::get_inverse_inertia_tensor(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	if (mode != PhysicsServer3D::BodyMode::BODY_MODE_RIGID) {
		return {};
	}

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetInverseInertia().GetQuaternion());
}

Vector3 JoltBody3D::get_linear_velocity(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetMotionPropertiesUnchecked()->GetLinearVelocity());
}

void JoltBody3D::set_linear_velocity(const Vector3& p_velocity, bool p_lock) {
	if (!space) {
		initial_linear_velocity = p_velocity;
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->GetMotionPropertiesUnchecked()->SetLinearVelocityClamped(to_jolt(p_velocity));
}

Vector3 JoltBody3D::get_angular_velocity(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetMotionPropertiesUnchecked()->GetAngularVelocity());
}

void JoltBody3D::set_angular_velocity(const Vector3& p_velocity, bool p_lock) {
	if (!space) {
		initial_angular_velocity = p_velocity;
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->GetMotionPropertiesUnchecked()->SetAngularVelocityClamped(to_jolt(p_velocity));
}

void JoltBody3D::set_center_of_mass_custom(const Vector3& p_center_of_mass, bool p_lock) {
	custom_center_of_mass = true;
	center_of_mass_custom = p_center_of_mass;
	rebuild_shape(p_lock);
}

void JoltBody3D::add_contact(
	const JoltBody3D* p_collider,
	float p_depth,
	int32_t p_shape_index,
	int32_t p_collider_shape_index,
	const Vector3& p_normal,
	const Vector3& p_position,
	const Vector3& p_collider_position,
	const Vector3& p_collider_velocity,
	float p_impulse
) {
	const int32_t max_contacts = get_max_contacts_reported();

	if (max_contacts == 0) {
		return;
	}

	Contact* contact = nullptr;

	if (contact_count != max_contacts) {
		contact = &contacts[contact_count++];
	} else {
		auto shallowest = std::min_element(
			contacts.begin(),
			contacts.end(),
			[](const Contact& p_lhs, const Contact& p_rhs) {
				return p_lhs.depth < p_rhs.depth;
			}
		);

		if (shallowest->depth < p_depth) {
			contact = &*shallowest;
		}
	}

	if (contact != nullptr) {
		contact->impulse = p_impulse;
		contact->shape_index = p_shape_index;
		contact->collider_shape_index = p_collider_shape_index;
		contact->collider_id = p_collider->get_instance_id();
		contact->collider_rid = p_collider->get_rid();
		contact->normal = p_normal;
		contact->position = p_position;
		contact->collider_position = p_collider_position;
		contact->collider_velocity = p_collider_velocity;
	}
}

void JoltBody3D::reset_mass_properties(bool p_lock) {
	inertia.zero();
	custom_center_of_mass = false;
	center_of_mass_custom.zero();
	mass_properties_changed(p_lock);
}

void JoltBody3D::apply_force(const Vector3& p_force, const Vector3& p_position, bool p_lock) {
	ERR_FAIL_NULL(space);

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	set_sleep_state(false, false);

	body->AddForce(to_jolt(p_force), body->GetCenterOfMassPosition() + to_jolt(p_position));
}

void JoltBody3D::apply_central_force(const Vector3& p_force, bool p_lock) {
	ERR_FAIL_NULL(space);

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	set_sleep_state(false, false);

	body->AddForce(to_jolt(p_force));
}

void JoltBody3D::apply_impulse(const Vector3& p_impulse, const Vector3& p_position, bool p_lock) {
	ERR_FAIL_NULL(space);

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	set_sleep_state(false, false);

	body->AddImpulse(to_jolt(p_impulse), body->GetCenterOfMassPosition() + to_jolt(p_position));
}

void JoltBody3D::apply_central_impulse(const Vector3& p_impulse, bool p_lock) {
	ERR_FAIL_NULL(space);

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	set_sleep_state(false, false);

	body->AddImpulse(to_jolt(p_impulse));
}

void JoltBody3D::apply_torque(const Vector3& p_torque, bool p_lock) {
	ERR_FAIL_NULL(space);

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	set_sleep_state(false, false);

	body->AddTorque(to_jolt(p_torque));
}

void JoltBody3D::apply_torque_impulse(const Vector3& p_impulse, bool p_lock) {
	ERR_FAIL_NULL(space);

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	set_sleep_state(false, false);

	body->AddAngularImpulse(to_jolt(p_impulse));
}

void JoltBody3D::add_constant_central_force(const Vector3& p_force) {
	constant_force += p_force;
}

void JoltBody3D::add_constant_force(
	const Vector3& p_force,
	const Vector3& p_position,
	bool p_lock
) {
	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	const Vector3 center_of_mass = get_center_of_mass(false);
	const Vector3 body_position = get_position(false);
	const Vector3 center_of_mass_relative = center_of_mass - body_position;

	constant_force += p_force;
	constant_torque += (p_position - center_of_mass_relative).cross(p_force);
}

void JoltBody3D::add_constant_torque(const Vector3& p_torque) {
	constant_torque += p_torque;
}

Vector3 JoltBody3D::get_constant_force() const {
	return constant_force;
}

void JoltBody3D::set_constant_force(const Vector3& p_force) {
	constant_force = p_force;
}

Vector3 JoltBody3D::get_constant_torque() const {
	return constant_torque;
}

void JoltBody3D::set_constant_torque(const Vector3& p_torque) {
	constant_torque = p_torque;
}

void JoltBody3D::add_collision_exception(const RID& p_excepted_body, bool p_lock) {
	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	// NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
	auto* group_filter = const_cast<JoltGroupFilterRID*>(
		static_cast<const JoltGroupFilterRID*>(body->GetCollisionGroup().GetGroupFilter())
	);

	if (group_filter == nullptr) {
		group_filter = new JoltGroupFilterRID();
		body->GetCollisionGroup().SetGroupFilter(group_filter);
	}

	group_filter->add_exception(p_excepted_body);
}

void JoltBody3D::remove_collision_exception(const RID& p_excepted_body, bool p_lock) {
	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	// NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
	auto* group_filter = const_cast<JoltGroupFilterRID*>(
		static_cast<const JoltGroupFilterRID*>(body->GetCollisionGroup().GetGroupFilter())
	);

	if (group_filter == nullptr) {
		return;
	}

	group_filter->remove_exception(p_excepted_body);

	if (group_filter->get_exception_count() == 0) {
		body->GetCollisionGroup().SetGroupFilter(nullptr);
		group_filter = nullptr;
	}
}

bool JoltBody3D::has_collision_exception(const RID& p_excepted_body, bool p_lock) const {
	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	const auto* group_filter =
		static_cast<const JoltGroupFilterRID*>(body->GetCollisionGroup().GetGroupFilter());

	return group_filter != nullptr && group_filter->has_exception(p_excepted_body);
}

TypedArray<RID> JoltBody3D::get_collision_exceptions(bool p_lock) const {
	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	const auto* group_filter =
		static_cast<const JoltGroupFilterRID*>(body->GetCollisionGroup().GetGroupFilter());

	if (group_filter == nullptr) {
		return {};
	}

	const RID* exceptions = group_filter->get_exceptions();
	const int32_t exception_count = group_filter->get_exception_count();

	TypedArray<RID> result;

	for (auto i = 0; i < exception_count; ++i) {
		result.push_back(exceptions[i]);
	}

	return result;
}

void JoltBody3D::add_area(JoltArea3D* p_area) {
	areas.ordered_insert(p_area, [](JoltArea3D* p_lhs, JoltArea3D* p_rhs) {
		return p_lhs->get_priority() > p_rhs->get_priority();
	});

	areas_changed();
}

void JoltBody3D::remove_area(JoltArea3D* p_area) {
	areas.erase(p_area);

	areas_changed();
}

void JoltBody3D::integrate_forces(float p_step, bool p_lock) {
	const JoltWritableBody3D jolt_body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(jolt_body.is_invalid());

	const Vector3 position = get_position(false);

	Vector3 total_gravity;

	bool gravity_done = false;

	for (JoltArea3D* area : areas) {
		gravity_done = integrate(total_gravity, area->get_gravity_mode(), [&]() {
			return area->compute_gravity(position);
		});

		if (gravity_done) {
			break;
		}
	}

	if (!gravity_done) {
		total_gravity += space->get_default_area()->compute_gravity(position);
	}

	total_gravity *= gravity_scale * p_step;

	JPH::MotionProperties& motion_properties = *jolt_body->GetMotionPropertiesUnchecked();

	motion_properties.SetLinearVelocityClamped(
		motion_properties.GetLinearVelocity() + to_jolt(total_gravity)
	);

	jolt_body->AddForce(to_jolt(constant_force));
	jolt_body->AddTorque(to_jolt(constant_torque));
}

void JoltBody3D::call_queries() {
	if (force_integration_callback.is_valid()) {
		static thread_local Array arguments = []() {
			Array array;
			array.resize(2);
			return array;
		}();

		arguments[0] = get_direct_state();
		arguments[1] = force_integration_userdata;

		force_integration_callback.callv(arguments);
	}

	if (body_state_callback.is_valid()) {
		static thread_local Array arguments = []() {
			Array array;
			array.resize(1);
			return array;
		}();

		arguments[0] = get_direct_state();

		body_state_callback.callv(arguments);
	}
}

void JoltBody3D::pre_step(float p_step) {
	JoltCollisionObject3D::pre_step(p_step);

	if (is_rigid()) {
		integrate_forces(p_step, false);
	}

	contact_count = 0;
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
		mode_changed(p_lock);
		return;
	}

	JPH::EMotionType motion_type = {};

	switch (mode) {
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
			ERR_FAIL_MSG(vformat("Unhandled body mode: '%d'", mode));
		} break;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	if (!get_sleep_state(false) && motion_type == JPH::EMotionType::Static) {
		set_sleep_state(true, false);
	}

	body->SetMotionType(motion_type);

	if (get_sleep_state(false) && motion_type != JPH::EMotionType::Static) {
		set_sleep_state(false, false);
	}

	if (motion_type == JPH::EMotionType::Kinematic) {
		body->SetLinearVelocity(JPH::Vec3::sZero());
		body->SetAngularVelocity(JPH::Vec3::sZero());
	}

	mode_changed(false);
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

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->SetRestitution(bounce);
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

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->SetFriction(friction);
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

	damp_changed(p_lock);
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

	damp_changed(p_lock);
}

JPH::EMotionType JoltBody3D::get_motion_type() const {
	switch (mode) {
		case PhysicsServer3D::BODY_MODE_STATIC: {
			return JPH::EMotionType::Static;
		}
		case PhysicsServer3D::BODY_MODE_KINEMATIC: {
			return JPH::EMotionType::Kinematic;
		}
		case PhysicsServer3D::BODY_MODE_RIGID:
		case PhysicsServer3D::BODY_MODE_RIGID_LINEAR: {
			return JPH::EMotionType::Dynamic;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled body mode: '%d'", mode));
		}
	}
}

void JoltBody3D::create_in_space(bool p_lock) {
	JPH::BodyCreationSettings settings = create_begin();

	settings.mLinearVelocity = to_jolt(initial_linear_velocity);
	settings.mAngularVelocity = to_jolt(initial_angular_velocity);
	settings.mAllowDynamicOrKinematic = true;
	settings.mMotionQuality =
		ccd_enabled ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
	settings.mAllowSleeping = allowed_sleep;
	settings.mFriction = friction;
	settings.mRestitution = bounce;
	settings.mGravityFactor = gravity_scale;
	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
	settings.mMassPropertiesOverride = calculate_mass_properties(*settings.GetShape());

	JPH::Body* body = create_end(settings, p_lock);

	// HACK(mihe): Since group filters don't grant us access to user data we are instead forced
	// abuse the collision group to carry the upper and lower bits of our RID, which we can then
	// access and rebuild in our group filter for bodies that make use of collision exceptions.

	JPH::CollisionGroup::GroupID group_id = 0;
	JPH::CollisionGroup::SubGroupID sub_group_id = 0;
	JoltGroupFilterRID::encode_rid(rid, group_id, sub_group_id);

	body->SetCollisionGroup(JPH::CollisionGroup(nullptr, group_id, sub_group_id));

	areas_changed(p_lock);
}

void JoltBody3D::mode_changed(bool p_lock) {
	object_layer_changed(p_lock);
}

void JoltBody3D::shapes_changed(bool p_lock) {
	mass_properties_changed(p_lock);
}

void JoltBody3D::areas_changed(bool p_lock) {
	damp_changed(p_lock);
}

void JoltBody3D::damp_changed(bool p_lock) {
	if (!space) {
		return;
	}

	float total_linear_damp = 0.0;
	float total_angular_damp = 0.0;

	bool linear_damp_done = linear_damp_mode == PhysicsServer3D::BODY_DAMP_MODE_REPLACE;
	bool angular_damp_done = angular_damp_mode == PhysicsServer3D::BODY_DAMP_MODE_REPLACE;

	for (JoltArea3D* area : areas) {
		if (!linear_damp_done) {
			linear_damp_done = integrate(total_linear_damp, area->get_linear_damp_mode(), [&]() {
				return area->get_linear_damp();
			});
		}

		if (!angular_damp_done) {
			angular_damp_done = integrate(total_angular_damp, area->get_angular_damp_mode(), [&]() {
				return area->get_angular_damp();
			});
		}

		if (linear_damp_done && angular_damp_done) {
			break;
		}
	}

	JoltArea3D* default_area = space->get_default_area();

	if (!linear_damp_done) {
		total_linear_damp += default_area->get_linear_damp();
	}

	if (!angular_damp_done) {
		total_angular_damp += default_area->get_angular_damp();
	}

	switch (linear_damp_mode) {
		case PhysicsServer3D::BODY_DAMP_MODE_COMBINE: {
			total_linear_damp += linear_damp;
		} break;
		case PhysicsServer3D::BODY_DAMP_MODE_REPLACE: {
			total_linear_damp = linear_damp;
		} break;
	}

	switch (angular_damp_mode) {
		case PhysicsServer3D::BODY_DAMP_MODE_COMBINE: {
			total_angular_damp += angular_damp;
		} break;
		case PhysicsServer3D::BODY_DAMP_MODE_REPLACE: {
			total_angular_damp = angular_damp;
		} break;
	}

	const JoltWritableBody3D jolt_body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(jolt_body.is_invalid());

	JPH::MotionProperties& motion_properties = *jolt_body->GetMotionPropertiesUnchecked();

	motion_properties.SetLinearDamping(total_linear_damp);
	motion_properties.SetAngularDamping(total_angular_damp);
}

JPH::MassProperties JoltBody3D::calculate_mass_properties(const JPH::Shape& p_shape) const {
	const bool calculate_mass = mass <= 0;
	const bool calculate_inertia = inertia.x <= 0 || inertia.y <= 0 || inertia.z <= 0;

	JPH::MassProperties mass_properties = p_shape.GetMassProperties();

	if (calculate_mass && calculate_inertia) {
		mass_properties.mInertia(3, 3) = 1.0f;
	} else if (calculate_inertia) {
		mass_properties.ScaleToMass(mass);
		mass_properties.mInertia(3, 3) = 1.0f;
	} else {
		mass_properties.mMass = mass;
		mass_properties.mInertia.SetDiagonal3(to_jolt(inertia));
	}

	return mass_properties;
}

JPH::MassProperties JoltBody3D::calculate_mass_properties() const {
	return calculate_mass_properties(*jolt_shape);
}

void JoltBody3D::mass_properties_changed(bool p_lock) {
	if (!space) {
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->GetMotionPropertiesUnchecked()->SetMassProperties(calculate_mass_properties());
}
