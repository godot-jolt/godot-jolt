#include "jolt_physics_collision_object_3d.hpp"

#include "conversion.hpp"
#include "jolt_physics_shape_3d.hpp"
#include "jolt_physics_space_3d.hpp"

JoltPhysicsCollisionObject3D::~JoltPhysicsCollisionObject3D() = default;

void JoltPhysicsCollisionObject3D::set_space(JoltPhysicsSpace3D* p_space) {
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

void JoltPhysicsCollisionObject3D::set_collision_layer(uint32_t p_layer) {
	if (space) {
		JPH::PhysicsSystem* system = space->get_system();
		const JPH::BodyLockInterface& lock_iface = system->GetBodyLockInterface();

		{
			const JPH::BodyLockWrite lock(lock_iface, jid);
			lock.GetBody().GetCollisionGroup().SetGroupID(p_layer);
		}
	}

	collision_layer = p_layer;
}

void JoltPhysicsCollisionObject3D::set_collision_mask(uint32_t p_mask) {
	if (space) {
		JPH::PhysicsSystem* system = space->get_system();
		const JPH::BodyLockInterface& lock_iface = system->GetBodyLockInterface();

		{
			const JPH::BodyLockWrite lock(lock_iface, jid);
			lock.GetBody().GetCollisionGroup().SetSubGroupID(p_mask);
		}
	}

	collision_mask = p_mask;
}

Transform3D JoltPhysicsCollisionObject3D::get_transform(bool p_lock) const {
	if (!space) {
		return transform;
	}

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface =
		p_lock ? system->GetBodyInterface() : system->GetBodyInterfaceNoLock();

	JPH::Vec3 position = {};
	JPH::Quat rotation = {};
	body_iface.GetPositionAndRotation(jid, position, rotation);

	return {to_godot(rotation), to_godot(position)};
}

void JoltPhysicsCollisionObject3D::set_transform(const Transform3D& p_transform) {
	transform = p_transform;

	if (!space) {
		return;
	}

	JPH::PhysicsSystem* system = space->get_system();
	JPH::BodyInterface& body_iface = system->GetBodyInterface();
	body_iface.SetPositionAndRotation(
		jid,
		to_jolt(p_transform.get_origin()),
		to_jolt(p_transform.get_basis()),
		JPH::EActivation::Activate
	);
}

Vector3 JoltPhysicsCollisionObject3D::get_center_of_mass() const {
	ERR_FAIL_COND_V_MSG(!space, Vector3(), "Object does not belong to any space.");

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface = system->GetBodyInterface();
	return to_godot(body_iface.GetCenterOfMassPosition(jid));
}

void JoltPhysicsCollisionObject3D::add_shape(
	JoltPhysicsShape3D* p_shape,
	const Transform3D& p_transform,
	bool p_disabled
) {
	Shape shape;
	shape.ref = p_shape;
	shape.transform = p_transform;
	shape.disabled = p_disabled;
	shapes.push_back(shape);

	p_shape->set_owner(this);

	if (JPH::MutableCompoundShape* root_shape = find_root_shape()) {
		root_shape->AddShape(
			to_jolt(p_transform.origin),
			to_jolt(p_transform.basis),
			p_shape->get_jref()
		);
	}
}

void JoltPhysicsCollisionObject3D::remove_shape(JoltPhysicsShape3D* p_shape) {
	const int index = find_shape_index(p_shape);
	if (index == -1) {
		return;
	}

	ERR_FAIL_INDEX(index, shapes.size());

	remove_shape(index);
}

void JoltPhysicsCollisionObject3D::remove_shape(int p_index) {
	const Shape& shape = shapes[p_index];
	shape.ref->set_owner(nullptr);
	shapes.remove_at(p_index);

	if (JPH::MutableCompoundShape* root_shape = find_root_shape()) {
		root_shape->RemoveShape((JPH::uint)p_index);
	}
}

int JoltPhysicsCollisionObject3D::find_shape_index(JoltPhysicsShape3D* p_shape) {
	for (int i = 0; i < shapes.size(); ++i) {
		if (shapes[i].ref == p_shape) {
			return i;
		}
	}

	return -1;
}

void JoltPhysicsCollisionObject3D::set_shape_transform(
	int64_t p_index,
	const Transform3D& p_transform
) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes.get((int)p_index).transform = p_transform;

	if (JPH::MutableCompoundShape* root_shape = find_root_shape()) {
		root_shape->ModifyShape(
			(JPH::uint)p_index,
			to_jolt(p_transform.origin),
			to_jolt(p_transform.basis)
		);
	}
}

JPH::MutableCompoundShape* JoltPhysicsCollisionObject3D::find_root_shape() const {
	if (!space) {
		return nullptr;
	}

	JPH::PhysicsSystem* system = space->get_system();
	const JPH::BodyInterface& body_iface = system->GetBodyInterface();
	const JPH::ShapeRefC root_shape_ref = body_iface.GetShape(jid);

	// HACK(mihe): const_cast is not ideal, but that's what the official tests for
	// MutableCompoundShape is using, as well as RefConst::InternalGetPtr
	// NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
	auto* root_shape = const_cast<JPH::Shape*>(root_shape_ref.GetPtr());

	// TODO(mihe): See if this could leverage JPH::DynamicCast instead
	return static_cast<JPH::MutableCompoundShape*>(root_shape);
}
