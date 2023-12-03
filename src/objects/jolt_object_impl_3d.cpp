#include "jolt_object_impl_3d.hpp"

#include "objects/jolt_group_filter.hpp"
#include "servers/jolt_project_settings.hpp"
#include "shapes/jolt_custom_empty_shape.hpp"
#include "shapes/jolt_shape_impl_3d.hpp"
#include "spaces/jolt_layer_mapper.hpp"
#include "spaces/jolt_space_3d.hpp"

JoltObjectImpl3D::JoltObjectImpl3D(ObjectType p_object_type)
	: object_type(p_object_type) {
	jolt_settings->mAllowSleeping = true;
	jolt_settings->mFriction = 1.0f;
	jolt_settings->mRestitution = 0.0f;
	jolt_settings->mLinearDamping = 0.0f;
	jolt_settings->mAngularDamping = 0.0f;
	jolt_settings->mGravityFactor = 1.0f;
}

JoltObjectImpl3D::~JoltObjectImpl3D() {
	delete_safely(jolt_settings);
}

GodotObject* JoltObjectImpl3D::get_instance() const {
	return internal::gdextension_interface_object_get_instance_from_id(instance_id);
}

Object* JoltObjectImpl3D::get_instance_unsafe() const {
	// HACK(mihe): This is being deliberately and incorrectly cast to a godot-cpp `Object` when in
	// reality it's a Godot `Object`. This is meant to be used in places where an `Object` is
	// returned through a parameter, such as in `PhysicsServer3DExtensionRayResult`, because
	// godot-cpp is unable to do the necessary unwrapping of the instance bindings in such cases.
	//
	// Dereferencing this pointer from the extension will lead to bad things.
	return reinterpret_cast<Object*>(get_instance());
}

Object* JoltObjectImpl3D::get_instance_wrapped() const {
	return ObjectDB::get_instance(instance_id);
}

void JoltObjectImpl3D::set_space(JoltSpace3D* p_space) {
	if (space == p_space) {
		return;
	}

	_space_changing();

	if (space != nullptr) {
		const JoltWritableBody3D body = space->write_body(jolt_id);
		ERR_FAIL_COND(body.is_invalid());

		jolt_settings = new JPH::BodyCreationSettings(body->GetBodyCreationSettings());

		_remove_from_space();
		_destroy_in_space();
	}

	space = p_space;

	if (space != nullptr) {
		_create_in_space();
		_add_to_space();
	}

	_space_changed();
}

void JoltObjectImpl3D::set_collision_layer(uint32_t p_layer) {
	if (p_layer == collision_layer) {
		return;
	}

	collision_layer = p_layer;

	_collision_layer_changed();
}

void JoltObjectImpl3D::set_collision_mask(uint32_t p_mask) {
	if (p_mask == collision_mask) {
		return;
	}

	collision_mask = p_mask;

	_collision_mask_changed();
}

Transform3D JoltObjectImpl3D::get_transform_unscaled() const {
	if (space == nullptr) {
		return {to_godot(jolt_settings->mRotation), to_godot(jolt_settings->mPosition)};
	}

	const JoltReadableBody3D body = space->read_body(jolt_id);
	ERR_FAIL_COND_D(body.is_invalid());

	return {to_godot(body->GetRotation()), to_godot(body->GetPosition())};
}

Transform3D JoltObjectImpl3D::get_transform_scaled() const {
	return get_transform_unscaled().scaled_local(scale);
}

void JoltObjectImpl3D::set_transform(Transform3D p_transform) {
	Vector3 new_scale;
	Math::decompose(p_transform, new_scale);

	// HACK(mihe): Ideally we would do an exact comparison here, but that would likely mismatch
	// quite often due to the nature of floating-point numbers. This does mean that the transform we
	// get back won't necessarily be the same one we set. We could solve this by storing the scale
	// regardless of approximate equality, but then we run the risk of many small changes creating a
	// large discrepancy between the transform reported and the one actually used by Jolt.
	if (!scale.is_equal_approx(new_scale)) {
		scale = new_scale;
		_shapes_changed();
	}

	_apply_transform(p_transform);

	_transform_changed();
}

Basis JoltObjectImpl3D::get_basis() const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mRotation);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetRotation());
}

Vector3 JoltObjectImpl3D::get_position() const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mPosition);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetPosition());
}

Vector3 JoltObjectImpl3D::get_center_of_mass() const {
	ERR_FAIL_NULL_D_MSG(
		space,
		vformat(
			"Failed to retrieve center-of-mass of '%s'. "
			"Doing so without a physics space is not supported by Godot Jolt. "
			"If this relates to a node, try adding the node to a scene tree first.",
			to_string()
		)
	);

	const JoltReadableBody3D body = space->read_body(jolt_id);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetCenterOfMassPosition());
}

Vector3 JoltObjectImpl3D::get_center_of_mass_local() const {
	ERR_FAIL_NULL_D_MSG(
		space,
		vformat(
			"Failed to retrieve local center-of-mass of '%s'. "
			"Doing so without a physics space is not supported by Godot Jolt. "
			"If this relates to a node, try adding the node to a scene tree first.",
			to_string()
		)
	);

	return get_transform_scaled().xform_inv(get_center_of_mass());
}

Vector3 JoltObjectImpl3D::get_linear_velocity() const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mLinearVelocity);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetLinearVelocity());
}

Vector3 JoltObjectImpl3D::get_angular_velocity() const {
	if (space == nullptr) {
		return to_godot(jolt_settings->mAngularVelocity);
	}

	const JoltReadableBody3D body = space->read_body(jolt_id);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetAngularVelocity());
}

JPH::ShapeRefC JoltObjectImpl3D::try_build_shape() {
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
		result = JoltShapeImpl3D::with_transform(
			last_built_shape->get_jolt_ref(),
			last_built_shape->get_transform_unscaled(),
			last_built_shape->get_scale()
		);
	} else {
		int32_t shape_index = 0;

		result = JoltShapeImpl3D::as_compound([&](auto&& p_add_shape) {
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
		result = JoltShapeImpl3D::with_center_of_mass(result, get_center_of_mass_custom());
	}

	if (scale != Vector3(1.0f, 1.0f, 1.0f)) {
		result = JoltShapeImpl3D::with_scale(result, scale);
	}

	return result;
}

void JoltObjectImpl3D::build_shape() {
	if (space == nullptr) {
		_shapes_built();
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id);
	ERR_FAIL_COND(body.is_invalid());

	previous_jolt_shape = jolt_shape;

	jolt_shape = try_build_shape();

	if (jolt_shape == nullptr) {
		jolt_shape = new JoltCustomEmptyShape();
	}

	if (jolt_shape == previous_jolt_shape) {
		return;
	}

	space->get_body_iface().SetShape(jolt_id, jolt_shape, false, JPH::EActivation::DontActivate);

	_shapes_built();
}

void JoltObjectImpl3D::add_shape(
	JoltShapeImpl3D* p_shape,
	Transform3D p_transform,
	bool p_disabled
) {
	Vector3 shape_scale;
	Math::decompose(p_transform, shape_scale);

	shapes.emplace_back(this, p_shape, p_transform, shape_scale, p_disabled);

	_shapes_changed();
}

void JoltObjectImpl3D::remove_shape(const JoltShapeImpl3D* p_shape) {
	shapes.erase_if([&](const JoltShapeInstance3D& p_instance) {
		return p_instance.get_shape() == p_shape;
	});

	_shapes_changed();
}

void JoltObjectImpl3D::remove_shape(int32_t p_index) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes.remove_at(p_index);

	_shapes_changed();
}

JoltShapeImpl3D* JoltObjectImpl3D::get_shape(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_shape();
}

void JoltObjectImpl3D::set_shape(int32_t p_index, JoltShapeImpl3D* p_shape) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes[p_index] = JoltShapeInstance3D(this, p_shape);

	_shapes_changed();
}

void JoltObjectImpl3D::clear_shapes() {
	shapes.clear();

	_shapes_changed();
}

int32_t JoltObjectImpl3D::find_shape_index(uint32_t p_shape_instance_id) const {
	return shapes.find_if([&](const JoltShapeInstance3D& p_shape) {
		return p_shape.get_id() == p_shape_instance_id;
	});
}

int32_t JoltObjectImpl3D::find_shape_index(const JPH::SubShapeID& p_sub_shape_id) const {
	ERR_FAIL_NULL_V(jolt_shape, -1);

	return find_shape_index((uint32_t)jolt_shape->GetSubShapeUserData(p_sub_shape_id));
}

JoltShapeImpl3D* JoltObjectImpl3D::find_shape(uint32_t p_shape_instance_id) const {
	const int32_t shape_index = find_shape_index(p_shape_instance_id);
	return shape_index != -1 ? shapes[shape_index].get_shape() : nullptr;
}

JoltShapeImpl3D* JoltObjectImpl3D::find_shape(const JPH::SubShapeID& p_sub_shape_id) const {
	const int32_t shape_index = find_shape_index(p_sub_shape_id);
	return shape_index != -1 ? shapes[shape_index].get_shape() : nullptr;
}

Transform3D JoltObjectImpl3D::get_shape_transform_unscaled(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_transform_unscaled();
}

Transform3D JoltObjectImpl3D::get_shape_transform_scaled(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_transform_scaled();
}

Vector3 JoltObjectImpl3D::get_shape_scale(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].get_scale();
}

void JoltObjectImpl3D::set_shape_transform(int32_t p_index, Transform3D p_transform) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	Vector3 new_scale;
	Math::decompose(p_transform, new_scale);

	JoltShapeInstance3D& shape = shapes[p_index];

	if (shape.get_transform_unscaled() == p_transform && shape.get_scale() == new_scale) {
		return;
	}

	shape.set_transform(p_transform);
	shape.set_scale(new_scale);

	_shapes_changed();
}

bool JoltObjectImpl3D::is_shape_disabled(int32_t p_index) const {
	ERR_FAIL_INDEX_D(p_index, shapes.size());

	return shapes[p_index].is_disabled();
}

void JoltObjectImpl3D::set_shape_disabled(int32_t p_index, bool p_disabled) {
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

	_shapes_changed();
}

void JoltObjectImpl3D::_add_to_space() {
	// HACK(mihe): Since `BODY_STATE_TRANSFORM` will be set right after creation it's more or less
	// impossible to have a body be sleeping when created, so we default to always starting out as
	// active.
	space->get_body_iface().AddBody(jolt_id, JPH::EActivation::Activate);
}

void JoltObjectImpl3D::_remove_from_space() {
	space->get_body_iface().RemoveBody(jolt_id);
}

void JoltObjectImpl3D::_destroy_in_space() {
	space->get_body_iface().DestroyBody(jolt_id);

	jolt_id = {};
}

void JoltObjectImpl3D::_apply_transform(const Transform3D& p_transform) {
	if (space == nullptr) {
		jolt_settings->mPosition = to_jolt(p_transform.origin);
		jolt_settings->mRotation = to_jolt(p_transform.basis);
		return;
	}

	space->get_body_iface().SetPositionAndRotation(
		jolt_id,
		to_jolt(p_transform.origin),
		to_jolt(p_transform.basis),
		JPH::EActivation::DontActivate
	);
}

void JoltObjectImpl3D::pre_step(
	[[maybe_unused]] float p_step,
	[[maybe_unused]] JPH::Body& p_jolt_body
) { }

void JoltObjectImpl3D::post_step(
	[[maybe_unused]] float p_step,
	[[maybe_unused]] JPH::Body& p_jolt_body
) {
	previous_jolt_shape = nullptr;
}

String JoltObjectImpl3D::to_string() const {
	Object* instance = ObjectDB::get_instance(instance_id);
	return instance != nullptr ? instance->to_string() : "<unknown>";
}

JPH::ObjectLayer JoltObjectImpl3D::_get_object_layer() const {
	if (space == nullptr) {
		return jolt_settings->mObjectLayer;
	}

	return space->map_to_object_layer(_get_broad_phase_layer(), collision_layer, collision_mask);
}

void JoltObjectImpl3D::_create_begin() {
	jolt_shape = try_build_shape();

	if (jolt_shape == nullptr) {
		jolt_shape = new JoltCustomEmptyShape();
	}

	JPH::CollisionGroup::GroupID group_id = 0;
	JPH::CollisionGroup::SubGroupID sub_group_id = 0;
	JoltGroupFilter::encode_object(this, group_id, sub_group_id);

	jolt_settings->mObjectLayer = _get_object_layer();
	jolt_settings->mCollisionGroup = JPH::CollisionGroup(nullptr, group_id, sub_group_id);
	jolt_settings->mMotionType = _get_motion_type();
	jolt_settings->SetShape(jolt_shape);
}

JPH::Body* JoltObjectImpl3D::_create_end() {
	ON_SCOPE_EXIT {
		delete_safely(jolt_settings);
	};

	JPH::Body* body = space->get_body_iface().CreateBody(*jolt_settings);

	ERR_FAIL_NULL_D_MSG(
		body,
		vformat(
			"Failed to create Jolt body for '%s'. "
			"Consider increasing maximum number of bodies in project settings. "
			"Maximum number of bodies is currently set to %d.",
			to_string(),
			JoltProjectSettings::get_max_bodies()
		)
	);

	body->SetUserData(reinterpret_cast<JPH::uint64>(this));

	jolt_id = body->GetID();

	return body;
}

void JoltObjectImpl3D::_update_object_layer() {
	if (space == nullptr) {
		return;
	}

	space->get_body_iface().SetObjectLayer(jolt_id, _get_object_layer());
}

void JoltObjectImpl3D::_collision_layer_changed() {
	_update_object_layer();
}

void JoltObjectImpl3D::_collision_mask_changed() {
	_update_object_layer();
}

void JoltObjectImpl3D::_shapes_changed() {
	build_shape();
}
