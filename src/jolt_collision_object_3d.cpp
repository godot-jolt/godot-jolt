#include "jolt_collision_object_3d.hpp"

#include "jolt_body_access_3d.hpp"
#include "jolt_layer_mapper.hpp"
#include "jolt_object_layer.hpp"
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

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());
	body_access.get_body().GetCollisionGroup().SetGroupID(p_layer);
}

void JoltCollisionObject3D::set_collision_mask(uint32_t p_mask, bool p_lock) {
	if (p_mask == collision_mask) {
		return;
	}

	collision_mask = p_mask;

	if (!space) {
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());
	body_access.get_body().GetCollisionGroup().SetSubGroupID(p_mask);
}

Transform3D JoltCollisionObject3D::get_transform(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	const JPH::Body& body = body_access.get_body();

	return {to_godot(body.GetRotation()), to_godot(body.GetPosition())};
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

Vector3 JoltCollisionObject3D::get_center_of_mass(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetCenterOfMassPosition());
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
	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	const JPH::Body& body = body_access.get_body();
	return calculate_mass_properties(*body.GetShape());
}

JPH::ShapeRefC JoltCollisionObject3D::try_build_shape() const {
	// TODO(mihe): This could greatly benefit from some kind of small/inlined vector, since I assume
	// most bodies will only ever have a handful of shapes
	Vector<JoltShapeInstance3D::Built> built_shapes;
	JoltShapeInstance3D::try_build(shapes, built_shapes);

	const int built_shape_count = built_shapes.size();

	if (built_shape_count == 0) {
		return {};
	}

	JPH::ShapeRefC result;

	if (built_shape_count == 1) {
		const JoltShapeInstance3D::Built& built_shape = built_shapes[0];
		const JoltShapeInstance3D& shape = *built_shape.shape;
		const JPH::ShapeRefC& jolt_ref = built_shape.jolt_ref;
		result = JoltShape3D::with_transform(jolt_ref, shape.get_transform());
	} else {
		result = JoltShapeInstance3D::build_compound(built_shapes);
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

	const JoltBodyAccessWrite3D body_access(*space, jolt_id, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::ShapeRefC shape = try_build_shape();

	JPH::BodyInterface& body_iface = space->get_body_iface(false);

	if (shape == nullptr) {
		shape = new JPH::SphereShape(1.0f);
		body_iface.SetObjectLayer(jolt_id, GDJOLT_OBJECT_LAYER_NONE);
	} else {
		const JPH::EMotionType motion_type = body_access.get_body().GetMotionType();
		const JPH::ObjectLayer object_layer = JoltLayerMapper::to_object_layer(motion_type);
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
	shapes.push_back({p_shape, p_transform, p_disabled});
	p_shape->add_owner(this);
	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::remove_shape(JoltShape3D* p_shape, bool p_lock) {
	const int32_t index = find_shape_index(p_shape);
	if (index >= 0) {
		remove_shape(index, p_lock);
	}
}

void JoltCollisionObject3D::remove_shape(int32_t p_index, bool p_lock) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes[p_index]->remove_owner(this);
	shapes.remove_at(p_index);
	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::remove_shapes(bool p_lock) {
	const int shape_count = shapes.size();

	for (int i = shape_count - 1; i >= 0; --i) {
		shapes[i]->remove_owner(this);
		shapes.remove_at(i);
	}

	rebuild_shape(p_lock);
}

int32_t JoltCollisionObject3D::find_shape_index(JoltShape3D* p_shape) {
	for (int32_t i = 0; i < shapes.size(); ++i) {
		if (shapes[i] == p_shape) {
			return i;
		}
	}

	return -1;
}

void JoltCollisionObject3D::set_shape_transform(
	int64_t p_index,
	const Transform3D& p_transform,
	bool p_lock
) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	const JoltShapeInstance3D& old_shape = shapes[(int32_t)p_index];

	if (old_shape.get_transform() == p_transform) {
		return;
	}

	shapes.write[(int32_t)p_index].set_transform(p_transform);

	rebuild_shape(p_lock);
}

void JoltCollisionObject3D::set_shape_disabled(int64_t p_index, bool p_disabled, bool p_lock) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	const JoltShapeInstance3D& old_shape = shapes[(int32_t)p_index];

	if (old_shape.is_disabled() == p_disabled) {
		return;
	}

	shapes.write[(int32_t)p_index].set_disabled(p_disabled);

	rebuild_shape(p_lock);
}
