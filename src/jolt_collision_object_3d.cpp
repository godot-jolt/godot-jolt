#include "jolt_collision_object_3d.hpp"

#include "jolt_layer_mapper.hpp"
#include "jolt_shape_3d.hpp"
#include "jolt_space_3d.hpp"

JoltCollisionObject3D::~JoltCollisionObject3D() = default;

void JoltCollisionObject3D::set_space(JoltSpace3D* p_space) {
	if (space == p_space) {
		return;
	}

	if (space) {
		space->remove_object(this);
		space->destroy_object(this);
		space = nullptr;
	}

	if (p_space) {
		p_space->create_object(this);
		p_space->add_object(this);
	}

	space = p_space;
}

void JoltCollisionObject3D::set_collision_layer(uint32_t p_layer, bool p_lock) {
	if (p_layer == collision_layer) {
		return;
	}

	collision_layer = p_layer;

	if (!space) {
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->GetCollisionGroup().SetGroupID(p_layer);
}

void JoltCollisionObject3D::set_collision_mask(uint32_t p_mask, bool p_lock) {
	if (p_mask == collision_mask) {
		return;
	}

	collision_mask = p_mask;

	if (!space) {
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	body->GetCollisionGroup().SetSubGroupID(p_mask);
}

Transform3D JoltCollisionObject3D::get_transform(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return {to_godot(body->GetRotation()), to_godot(body->GetPosition())};
}

void JoltCollisionObject3D::set_transform(const Transform3D& p_transform, bool p_lock) {
	if (!space) {
		initial_transform = p_transform;
		return;
	}

	space->get_body_iface(p_lock).SetPositionAndRotation(
		jolt_id,
		to_jolt(p_transform.get_origin()),
		to_jolt(p_transform.get_basis()),
		JPH::EActivation::DontActivate
	);
}

Basis JoltCollisionObject3D::get_basis(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetRotation());
}

void JoltCollisionObject3D::set_basis(const Basis& p_basis, bool p_lock) {
	if (!space) {
		initial_transform.basis = p_basis;
		return;
	}

	space->get_body_iface(p_lock)
		.SetRotation(jolt_id, to_jolt(p_basis), JPH::EActivation::DontActivate);
}

Vector3 JoltCollisionObject3D::get_position(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetPosition());
}

void JoltCollisionObject3D::set_position(const Vector3& p_position, bool p_lock) {
	if (!space) {
		initial_transform.origin = p_position;
		return;
	}

	space->get_body_iface(p_lock)
		.SetPosition(jolt_id, to_jolt(p_position), JPH::EActivation::DontActivate);
}

Vector3 JoltCollisionObject3D::get_center_of_mass(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetCenterOfMassPosition());
}

Vector3 JoltCollisionObject3D::get_velocity_at_local_position(
	const Vector3& p_position,
	bool p_lock
) const {
	ERR_FAIL_NULL_D(space);

	const JoltReadableBody3D body = space->read_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return to_godot(body->GetPointVelocity(body->GetWorldTransform() * to_jolt(p_position)));
}

JPH::MassProperties JoltCollisionObject3D::calculate_mass_properties(const JPH::Shape& p_shape
) const {
	const float mass = get_mass();
	const Vector3 inertia = get_inertia();

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

JPH::MassProperties JoltCollisionObject3D::calculate_mass_properties(bool p_lock) const {
	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND_D(body.is_invalid());

	return calculate_mass_properties(*body->GetShape());
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
			last_built_shape->get_transform()
		);
	} else {
		int32_t shape_index = 0;

		result = JoltShape3D::as_compound([&](auto&& p_add_shape) {
			if (shape_index >= shapes.size()) {
				return false;
			}

			const JoltShapeInstance3D& shape = shapes[shape_index++];

			if (shape.is_enabled() && shape.is_built()) {
				p_add_shape(shape.get_jolt_ref(), shape.get_transform());
			}

			return true;
		});
	}

	if (has_custom_center_of_mass()) {
		result = JoltShape3D::with_center_of_mass(result, get_center_of_mass_custom());
	}

	return result;
}

void JoltCollisionObject3D::rebuild_shape(bool p_lock) {
	if (!space) {
		shapes_changed(p_lock);
		return;
	}

	const JoltWritableBody3D body = space->write_body(jolt_id, p_lock);
	ERR_FAIL_COND(body.is_invalid());

	JPH::ShapeRefC shape = try_build_shape();

	JPH::BodyInterface& body_iface = space->get_body_iface(false);

	if (shape == nullptr) {
		// Use a fallback shape instead
		shape = new JPH::SphereShape(1.0f);

		// Place it in object (and broad phase) layer 0, which will make it collide with nothing
		body_iface.SetObjectLayer(jolt_id, 0);
	} else {
		const JPH::ObjectLayer object_layer = space->map_to_object_layer(
			body->GetMotionType(),
			get_collision_layer(),
			get_collision_mask()
		);

		body_iface.SetObjectLayer(jolt_id, object_layer);
	}

	body_iface.SetShape(jolt_id, shape, false, JPH::EActivation::DontActivate);

	shapes_changed(false);
}

void JoltCollisionObject3D::add_shape(
	JoltShape3D* p_shape,
	const Transform3D& p_transform,
	bool p_disabled,
	bool p_lock
) {
	shapes.emplace_back(this, p_shape, p_transform, p_disabled);

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

int32_t JoltCollisionObject3D::find_shape_index(uint32_t p_shape_instance_id) const {
	return shapes.find_if([&](const JoltShapeInstance3D& p_shape) {
		return p_shape.get_id() == p_shape_instance_id;
	});
}

void JoltCollisionObject3D::set_shape_transform(
	int32_t p_index,
	const Transform3D& p_transform,
	bool p_lock
) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	JoltShapeInstance3D& shape = shapes[p_index];

	if (shape.get_transform() == p_transform) {
		return;
	}

	shape.set_transform(p_transform);

	rebuild_shape(p_lock);
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
