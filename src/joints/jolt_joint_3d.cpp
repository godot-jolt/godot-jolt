#include "jolt_joint_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr int32_t DEFAULT_SOLVER_PRIORITY = 1;

} // namespace

JoltJoint3D::JoltJoint3D(JoltSpace3D* p_space, JoltBody3D* p_body_a, JoltBody3D* p_body_b)
	: space(p_space)
	, body_a(p_body_a)
	, body_b(p_body_b) { }

JoltJoint3D::~JoltJoint3D() {
	if (jolt_ref != nullptr) {
		space->remove_joint(this);
	}
}

int32_t JoltJoint3D::get_solver_priority() const {
	return DEFAULT_SOLVER_PRIORITY;
}

void JoltJoint3D::set_solver_priority(int32_t p_priority) {
	if (p_priority != DEFAULT_SOLVER_PRIORITY) {
		WARN_PRINT(
			"Joint solver priority is not supported by Godot Jolt. "
			"Any such value will be ignored."
		);
	}
}

void JoltJoint3D::set_collision_disabled(bool p_disabled) {
	collision_disabled = p_disabled;

	if (body_b == nullptr) {
		return;
	}

	PhysicsServer3D* physics_server = PhysicsServer3D::get_singleton();

	if (collision_disabled) {
		physics_server->body_add_collision_exception(body_a->get_rid(), body_b->get_rid());
		physics_server->body_add_collision_exception(body_b->get_rid(), body_a->get_rid());
	} else {
		physics_server->body_remove_collision_exception(body_a->get_rid(), body_b->get_rid());
		physics_server->body_remove_collision_exception(body_b->get_rid(), body_a->get_rid());
	}
}
