#include "jolt_collision_object_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "jolt_body_access_3d.hpp"
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

	const JoltBodyAccessWrite3D body_access(*space, jid, p_lock);
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

	const JoltBodyAccessWrite3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());
	body_access.get_body().GetCollisionGroup().SetSubGroupID(p_mask);
}

Transform3D JoltCollisionObject3D::get_transform(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	const JPH::Body& body = body_access.get_body();

	return {to_godot(body.GetRotation()), to_godot(body.GetPosition())};
}

void JoltCollisionObject3D::set_transform(const Transform3D& p_transform, bool p_lock) {
	if (!space) {
		initial_transform = p_transform;
		return;
	}

	// We need to go through `JPH::BodyInterface` instead of `JPH::Body`, because the latter doesn't
	// allow us to set position/rotation, presumably due to needing to modify the broad phase

	space->get_body_iface(p_lock).SetPositionAndRotation(
		jid,
		to_jolt(p_transform.get_origin()),
		to_jolt(p_transform.get_basis()),
		JPH::EActivation::DontActivate
	);
}

Vector3 JoltCollisionObject3D::get_center_of_mass(bool p_lock) const {
	ERR_FAIL_NULL_D(space);

	const JoltBodyAccessRead3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	return to_godot(body_access.get_body().GetCenterOfMassPosition());
}

JPH::MassProperties JoltCollisionObject3D::calculate_mass_properties(const JPH::Shape& p_root_shape
) const {
	const float mass = get_mass();
	const Vector3 inertia = get_inertia();

	const bool calculate_mass = mass <= 0;
	const bool calculate_inertia = inertia.x <= 0 || inertia.y <= 0 || inertia.z <= 0;

	JPH::MassProperties mass_properties = p_root_shape.GetMassProperties();

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
	const JoltBodyAccessWrite3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND_D(!body_access.is_valid());

	const JPH::Body& body = body_access.get_body();
	return calculate_mass_properties(*body.GetShape());
}

void JoltCollisionObject3D::add_shape(
	JoltShape3D* p_shape,
	const Transform3D& p_transform,
	bool p_disabled,
	bool p_lock
) {
	Shape shape;
	shape.ref = p_shape;
	shape.transform = p_transform;
	shape.disabled = p_disabled;
	shapes.push_back(shape);

	p_shape->set_owner(this);

	if (!space) {
		shapes_changed(p_lock);
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::MutableCompoundShape* root_shape = get_root_shape();
	ERR_FAIL_NULL(root_shape);

	const JPH::Vec3 previous_com = root_shape->GetCenterOfMass();

	root_shape
		->AddShape(to_jolt(p_transform.origin), to_jolt(p_transform.basis), p_shape->get_jref());

	space->get_body_iface(false)
		.NotifyShapeChanged(jid, previous_com, false, JPH::EActivation::DontActivate);

	shapes_changed(false);
}

void JoltCollisionObject3D::remove_shape(JoltShape3D* p_shape, bool p_lock) {
	const int index = find_shape_index(p_shape);
	if (index == -1) {
		return;
	}

	remove_shape(index, p_lock);
}

void JoltCollisionObject3D::remove_shape(int p_index, bool p_lock) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	const Shape& shape = shapes[p_index];
	shape.ref->set_owner(nullptr);
	shapes.remove_at(p_index);

	if (!space) {
		shapes_changed(p_lock);
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::MutableCompoundShape* root_shape = get_root_shape();
	ERR_FAIL_NULL(root_shape);
	ERR_FAIL_INDEX(p_index, (int)root_shape->GetNumSubShapes());

	const JPH::Vec3 previous_com = root_shape->GetCenterOfMass();

	root_shape->RemoveShape((JPH::uint)p_index);

	space->get_body_iface(false)
		.NotifyShapeChanged(jid, previous_com, false, JPH::EActivation::DontActivate);

	shapes_changed(false);
}

int JoltCollisionObject3D::find_shape_index(JoltShape3D* p_shape) {
	for (int i = 0; i < shapes.size(); ++i) {
		if (shapes[i].ref == p_shape) {
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

	shapes.get((int)p_index).transform = p_transform;

	if (!space) {
		shapes_changed(p_lock);
		return;
	}

	const JoltBodyAccessWrite3D body_access(*space, jid, p_lock);
	ERR_FAIL_COND(!body_access.is_valid());

	JPH::MutableCompoundShape* root_shape = get_root_shape();
	ERR_FAIL_NULL(root_shape);
	ERR_FAIL_INDEX(p_index, (int)root_shape->GetNumSubShapes());

	const JPH::Vec3 previous_com = root_shape->GetCenterOfMass();

	root_shape
		->ModifyShape((JPH::uint)p_index, to_jolt(p_transform.origin), to_jolt(p_transform.basis));

	space->get_body_iface(false)
		.NotifyShapeChanged(jid, previous_com, false, JPH::EActivation::DontActivate);

	shapes_changed(false);
}

JPH::MutableCompoundShape* JoltCollisionObject3D::get_root_shape() const {
	ERR_FAIL_NULL_D(space);

	// We assume that we're already locked, since anything returned from this function after the
	// lock is released could disappear from underneath us anyway
	const JoltBodyAccessRead3D body_access(*space, jid, false);
	ERR_FAIL_COND_D(!body_access.is_valid());

	// HACK(mihe): const_cast is not ideal, but that's what the official tests for
	// `MutableCompoundShape` are using, as well as `RefConst::InternalGetPtr`
	// NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
	auto* root_shape = const_cast<JPH::Shape*>(body_access.get_body().GetShape());

	return static_cast<JPH::MutableCompoundShape*>(root_shape);
}
