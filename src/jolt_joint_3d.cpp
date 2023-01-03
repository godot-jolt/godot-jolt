#include "jolt_joint_3d.hpp"

#include "jolt_space_3d.hpp"

namespace {

constexpr int64_t GDJOLT_JOINT_DEFAULT_SOLVER_PRIORITY = 1;

} // namespace

JoltJoint3D::JoltJoint3D(JoltSpace3D* p_space)
	: space(p_space) { }

JoltJoint3D::~JoltJoint3D() {
	if (jolt_ref != nullptr) {
		space->remove_joint(this);
	}
}

int64_t JoltJoint3D::get_solver_priority() const {
	return GDJOLT_JOINT_DEFAULT_SOLVER_PRIORITY;
}

void JoltJoint3D::set_solver_priority(int64_t p_priority) {
	if (p_priority != GDJOLT_JOINT_DEFAULT_SOLVER_PRIORITY) {
		WARN_PRINT(
			"Joint solver priority is not supported by Godot Jolt. "
			"Any such value will be ignored."
		);
	}
}
