#include "jolt_pin_joint_3d.hpp"

#include "servers/jolt_physics_server_3d.hpp"

namespace {

JoltPhysicsServer3D* get_physics_server() {
	static auto* singleton = dynamic_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());
	return singleton;
}

} // namespace

void JoltPinJoint3D::_bind_methods() {
	BIND_METHOD(JoltPinJoint3D, get_linear_impulse);
}

Vector3 JoltPinJoint3D::get_linear_impulse() const {
	JoltPhysicsServer3D* physics_server = get_physics_server();
	ERR_FAIL_NULL_D(physics_server);

	return physics_server->pin_joint_get_linear_impulse(rid);
}

void JoltPinJoint3D::_configure(PhysicsBody3D* p_body_a, PhysicsBody3D* p_body_b) {
	JoltPhysicsServer3D* physics_server = get_physics_server();
	ERR_FAIL_NULL(physics_server);

	const Vector3 global_position = get_global_position();

	physics_server->joint_make_pin(
		rid,
		p_body_a->get_rid(),
		p_body_a->to_local(global_position),
		p_body_b != nullptr ? p_body_b->get_rid() : RID(),
		p_body_b != nullptr ? p_body_b->to_local(global_position) : global_position
	);
}
