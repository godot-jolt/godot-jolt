#include "jolt_collision_object_3d.hpp"

#include "servers/jolt_project_settings.hpp"
#include "shapes/jolt_empty_shape.hpp"
#include "shapes/jolt_shape_3d.hpp"
#include "spaces/jolt_layer_mapper.hpp"
#include "spaces/jolt_space_3d.hpp"

JoltCollisionObject3D::JoltCollisionObject3D() {
	jolt_settings->mAllowSleeping = true;
	jolt_settings->mFriction = 1.0f;
	jolt_settings->mRestitution = 0.0f;
	jolt_settings->mLinearDamping = 0.0f;
	jolt_settings->mAngularDamping = 0.0f;
	jolt_settings->mGravityFactor = 1.0f;
}

JoltCollisionObject3D::~JoltCollisionObject3D() {
	delete_safely(jolt_settings);
}

GodotObject* JoltCollisionObject3D::get_instance() const {
	return internal::gde_interface->object_get_instance_from_id(instance_id);
}

Object* JoltCollisionObject3D::get_instance_unsafe() const {
	// HACK(mihe): This is being deliberately and incorrectly cast to a godot-cpp `Object` when in
	// reality it's a Godot `Object`. This is meant to be used in places where an `Object` is
	// returned through a parameter, such as in `PhysicsServer3DExtensionRayResult`, because
	// godot-cpp is unable to do the necessary unwrapping of the instance bindings in such cases.
	//
	// Dereferencing this pointer from the extension will lead to bad things.
	return reinterpret_cast<Object*>(get_instance());
}

Object* JoltCollisionObject3D::get_instance_wrapped() const {
	return ObjectDB::get_instance(instance_id);
}

void JoltCollisionObject3D::set_space(JoltSpace3D* p_space, bool p_lock) {
	if (space == p_space) {
		return;
	}

	space_changing(p_lock);

	if (space != nullptr) {
		const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
		ERR_FAIL_COND(body.is_invalid());

		jolt_settings = new JPH::BodyCreationSettings(body->GetBodyCreationSettings());

		remove_from_space(false);
		destroy_in_space(false);
	}

	space = p_space;

	if (space != nullptr) {
		create_in_space();
		add_to_space();
	}

	space_changed(p_lock);
}

void JoltCollisionObject3D::set_collision_layer(uint32_t p_layer, bool p_lock) {
	if (p_layer == collision_layer) {
		return;
	}

	collision_layer = p_layer;

	collision_layer_changed(p_lock);
}

void JoltCollisionObject3D::set_collision_mask(uint32_t p_mask, bool p_lock) {
	if (p_mask == collision_mask) {
		return;
	}

	collision_mask = p_mask;

	collision_mask_changed(p_lock);
}

Transform3D JoltCollisionObject3D::get_transform_unscaled(bool p_lock) const {
	if (space == nullptr) {
		return {to_godot(jolt_settings->mRotation), to_godot(jolt_settings->mPosition)};
	}

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return {to_godot(body->GetRotation()), to_godot(body->GetPosition())};
}

Transform3D JoltCollisionObject3D::get_transform_scaled(bool p_lock) const {
	return get_transform_unscaled(p_lock).scaled_local(scale);
}

void JoltCollisionObject3D::set_transform(Transform3D p_transform, bool p_lock) {
	Vector3 new_scale;
	Math::decompose(p_transform, new_scale);

	// HACK(mihe): Ideally we would do an exact comparison here, but that would likely mismatch
	// quite often due to the nature of floating-point numbers. This does mean that the transform we
	// get back won't necessarily be the same one we set. We could solve this by storing the scale
	// regardless of approximate equality, but then we run the risk of many small changes creating a
	// large discrepancy between the transform reported and the one actually used by Jolt.
	if (!scale.is_equal_approx(new_scale)) {
		scale = new_scale;
		rebuild_shape(p_lock);
	}

	if (space == nullptr) {
		jolt_settings->mPosition = to_jolt(p_transform.origin);
		jolt_settings->mRotation = to_jolt(p_transform.basis);
		transform_changed(p_lock);
		return;
	}

	space->get_body_iface(p_lock).SetPositionAndRotation(
		jolt_id,
		to_jolt(p_transform.origin),
		to_jolt(p_transform.basis),
		JPH::EActivation::DontActivate
	);

	transform_changed(p_lock);
}

Basis JoltCollisionObject3D::get_basis(bool p_lock) const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mRotation);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetRotation());
}

Vector3 JoltCollisionObject3D::get_position(bool p_lock) const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mPosition);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetPosition());
}

Vector3 JoltCollisionObject3D::get_center_of_mass(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetCenterOfMassPosition());
}

Vector3 JoltCollisionObject3D::get_center_of_mass_local(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	return get_transform_scaled(p_lock).xform_inv(get_center_of_mass(p_lock));
}

Vector3 JoltCollisionObject3D::get_linear_velocity(bool p_lock) const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mLinearVelocity);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetLinearVelocity());
}

Vector3 JoltCollisionObject3D::get_angular_velocity(bool p_lock) const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mAngularVelocity);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetAngularVelocity());
}

Vector3 JoltCollisionObject3D::get_velocity_at_position(const Vector3& p_position, bool p_lock)
	const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetPointVelocity(to_jolt(p_position)));
}

JPH::ShapeRefC JoltCollisionObject3D::try_build_shape() {
	int32_t built_shape_count = 0;
	const JoltShapeInstance3D* last_built_shape = nullptr;

	for (JoltShapeInstance3D& shape : shapes) {
		if (shape.is_enabled() && shape.try_build()) {
			built_shape_count += 1;
			last_built_shape = &shape;
		}
	}

	if (built_shape_count == 0) {
		return {};
	}

	JPH::ShapeRefC result;

	if (built_shape_count == 1) {
		result = JoltShape3D::with_transform(
			last_built_shape->get_jolt_ref(),
			last_built_shape->get_transform_unscaled(),
			last_built_shape->get_scale()
		);
	} else {
		int32_t shape_index = 0;

		result = JoltShape3D::as_compound([&](auto&& p_add_shape) {
			if (shape_index >= shapes.size()) {
				return false;
			}

			const JoltShapeInstance3D& shape = shapes[shape_index++];

			if (shape.is_enabled() && shape.is_built()) {
				p_add_shape(
					shape.get_jolt_ref(),
					shape.get_transform_unscaled(),
					shape.get_scale()
				);
			}

			return true;
		});
	}

	if (has_custom_center_of_mass()) {
		result = JoltShape3D::with_center_of_mass(result, get_center_of_mass_custom());
	}

	if (scale != Vector3(1.0f, 1.0f, 1.0f)) {
		result = JoltShape3D::with_scale(result, scale);
	}

	return result;
}

void JoltCollisionObject3D::rebuild_shape(bool p_lock) {
	if (space == nullptr) {
		shapes_changed(p_lock);
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	previous_jolt_shape = jolt_shape;

	jolt_shape = try_build_shape();

	if (jolt_shape == nullptr) {
		jolt_shape = new JoltEmptyShape();
	}

	if (jolt_shape == previous_jolt_shape) {
		return;
	}

	space->get_body_iface(false)
		.SetShape(jolt_id, jolt_shape, false, JPH::EActivation::DontActivate);

	shapes_changed(false);
}

void JoltCollisionObject3D::add_shape(
	JoltShape3D* p_shape,
	Transform3D p_transform,
	bool p_disabled,
	bool p_lock
) {
	Vector3 shape_scale;
	Math::decompose(p_transform, shape_scale);

	shapes.emplace_back(this, p_shape, p_transform, shape_scale, p_disabled);

	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::remove_shape(JoltShape3D* p_shape, bool p_lock) {
	shapes.erase_if([&](const JoltShapeInstance3D& p_instance) {
		return p_instance.get_shape() == p_shape;
	});

	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::remove_shape(int32_t p_index, bool p_lock) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes.remove_at(p_index);

	rebuild_shape(p_lock);
}

JoltShape3D* JoltCollisionObject3D::get_shape(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_shape();
}

void JoltCollisionObject3D::set_shape(int32_t p_index, JoltShape3D* p_shape, bool p_lock) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes[p_index] = JoltShapeInstance3D(this, p_shape);

	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::clear_shapes(bool p_lock) {
	shapes.clear();

	rebuild_shape(p_lock);
}

int32_t JoltCollisionObject3D::find_shape_index(uint32_t p_shape_instance_id) const {
	return shapes.find_if([&](const JoltShapeInstance3D& p_shape) {
		return p_shape.get_id() == p_shape_instance_id;
	});
}

int32_t JoltCollisionObject3D::find_shape_index(const JPH::SubShapeID& p_sub_shape_id) const {
	ERR_FAIL_NULL_V(jolt_shape, -1);

	return find_shape_index((uint32_t)jolt_shape->GetSubShapeUserData(p_sub_shape_id));
}

JoltShape3D* JoltCollisionObject3D::find_shape(uint32_t p_shape_instance_id) const {
	const int32_t shape_index = find_shape_index(p_shape_instance_id);
	return shape_index != -1 ? shapes[shape_index].get_shape() : nullptr;
}

JoltShape3D* JoltCollisionObject3D::find_shape(const JPH::SubShapeID& p_sub_shape_id) const {
	const int32_t shape_index = find_shape_index(p_sub_shape_id);
	return shape_index != -1 ? shapes[shape_index].get_shape() : nullptr;
}

Transform3D JoltCollisionObject3D::get_shape_transform_unscaled(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_transform_unscaled();
}

Transform3D JoltCollisionObject3D::get_shape_transform_scaled(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_transform_scaled();
}

Vector3 JoltCollisionObject3D::get_shape_scale(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_scale();
}

void JoltCollisionObject3D::set_shape_transform(
	int32_t p_index,
	Transform3D p_transform,
	bool p_lock
) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	Vector3 new_scale;
	Math::decompose(p_transform, new_scale);

	JoltShapeInstance3D& shape = shapes[p_index];

	if (shape.get_transform_unscaled() == p_transform && shape.get_scale() == new_scale) {
		return;
	}

	shape.set_transform(p_transform);
	shape.set_scale(new_scale);

	rebuild_shape(p_lock);
}

bool JoltCollisionObject3D::is_shape_disabled(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].is_disabled();
}

void JoltCollisionObject3D::set_shape_disabled(int32_t p_index, bool p_disabled, bool p_lock) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	JoltShapeInstance3D& shape = shapes[p_index];

	if (shape.is_disabled() == p_disabled) {
		return;
	}

	if (p_disabled) {
		shape.disable();
	} else {
		shape.enable();
	}

	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::add_to_space(bool p_lock) {
	// HACK(mihe): Since `BODY_STATE_TRANSFORM` will be set right after creation it's more or less
	// impossible to have a body be sleeping when created, so we default to always starting out as
	// active.
	space->get_body_iface(p_lock).AddBody(jolt_id, JPH::EActivation::Activate);
}

void JoltCollisionObject3D::remove_from_space(bool p_lock) {
	space->get_body_iface(p_lock).RemoveBody(jolt_id);
}

void JoltCollisionObject3D::destroy_in_space(bool p_lock) {
	space->get_body_iface(p_lock).DestroyBody(jolt_id);

	jolt_id = {};
}

void JoltCollisionObject3D::pre_step([[maybe_unused]] float p_step) { }

void JoltCollisionObject3D::post_step([[maybe_unused]] float p_step) {
	previous_jolt_shape = nullptr;
}

JPH::ObjectLayer JoltCollisionObject3D::get_object_layer() const {
	if (space == nullptr) {
		return jolt_settings->mObjectLayer;
	}

	return space->map_to_object_layer(get_broad_phase_layer(), collision_layer, collision_mask);
}

void JoltCollisionObject3D::create_begin() {
	jolt_shape = try_build_shape();

	if (jolt_shape == nullptr) {
		jolt_shape = new JoltEmptyShape();
	}

	jolt_settings->mObjectLayer = get_object_layer();
	jolt_settings->mMotionType = get_motion_type();
	jolt_settings->SetShape(jolt_shape);
}

JPH::Body* JoltCollisionObject3D::create_end() {
	ON_SCOPE_EXIT {
		delete_safely(jolt_settings);
	};

	JPH::Body* body = space->get_body_iface(false).CreateBody(*jolt_settings);

	ERR_FAIL_NULL_D_MSG(
		body,
		vformat(
			"Failed to create Jolt body. "
			"Consider increasing maximum number of bodies in project settings. "
			"Maximum number of bodies is currently set to %d.",
			JoltProjectSettings::get_max_bodies()
		)
	);

	body->SetUserData(reinterpret_cast<JPH::uint64>(this));

	jolt_id = body->GetID();

	return body;
}

void JoltCollisionObject3D::update_object_layer(bool p_lock) {
	if (space == nullptr) {
		return;
	}

	space->get_body_iface(p_lock).SetObjectLayer(jolt_id, get_object_layer());
}

void JoltCollisionObject3D::collision_layer_changed(bool p_lock) {
	update_object_layer(p_lock);
}

void JoltCollisionObject3D::collision_mask_changed(bool p_lock) {
	update_object_layer(p_lock);
}
