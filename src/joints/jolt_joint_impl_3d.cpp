#include "jolt_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr int32_t DEFAULT_SOLVER_PRIORITY = 1;

} // namespace

JoltJointImpl3D::JoltJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b
)
	: body_a(p_body_a)
	, body_b(p_body_b)
	, local_ref_a(p_local_ref_a)
	, local_ref_b(p_local_ref_b) {
	body_a->add_joint(this);

	if (body_b != nullptr) {
		body_b->add_joint(this);
	}
}

JoltJointImpl3D::~JoltJointImpl3D() {
	if (body_a != nullptr) {
		body_a->remove_joint(this);
	}

	if (body_b != nullptr) {
		body_b->remove_joint(this);
	}

	destroy();
}

JoltSpace3D* JoltJointImpl3D::get_space() const {
	JoltSpace3D* space_a = body_a->get_space();

	if (body_b != nullptr) {
		JoltSpace3D* space_b = body_b->get_space();
		QUIET_FAIL_NULL_D(space_b);

		ERR_FAIL_COND_D_MSG(
			space_a != space_b,
			vformat(
				"Joint was found to connect bodies in different physics spaces. "
				"This joint will effectively be disabled. "
				"This joint connects %s.",
				bodies_to_string()
			)
		);
	}

	return space_a;
}

int32_t JoltJointImpl3D::get_solver_priority() const {
	return DEFAULT_SOLVER_PRIORITY;
}

void JoltJointImpl3D::set_solver_priority(int32_t p_priority) {
	if (p_priority != DEFAULT_SOLVER_PRIORITY) {
		WARN_PRINT(vformat(
			"Joint solver priority is not supported by Godot Jolt. "
			"Any such value will be ignored."
			"This joint connects %s.",
			bodies_to_string()
		));
	}
}

void JoltJointImpl3D::set_collision_disabled(bool p_disabled) {
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

void JoltJointImpl3D::destroy() {
	if (jolt_ref == nullptr) {
		return;
	}

	JoltSpace3D* space = get_space();

	if (space != nullptr) {
		space->remove_joint(this);
	}

	jolt_ref = nullptr;
}

void JoltJointImpl3D::shift_reference_frames(
	const Vector3& p_linear_shift,
	const Vector3& p_angular_shift,
	Transform3D& p_shifted_ref_a,
	Transform3D& p_shifted_ref_b
) {
	Vector3 origin_a = local_ref_a.origin;
	Vector3 origin_b = local_ref_b.origin;

	origin_a *= body_a->get_scale();
	origin_a -= to_godot(body_a->get_jolt_shape()->GetCenterOfMass());

	if (body_b != nullptr) {
		origin_b *= body_b->get_scale();
		origin_b -= to_godot(body_b->get_jolt_shape()->GetCenterOfMass());
	}

	const Basis& basis_a = local_ref_a.basis;
	const Basis& basis_b = local_ref_b.basis;

	const Basis shifted_basis_a = basis_a * Basis::from_euler(p_angular_shift);
	const Vector3 shifted_origin_a = origin_a - basis_a.xform(p_linear_shift);

	p_shifted_ref_a = Transform3D(shifted_basis_a, shifted_origin_a);
	p_shifted_ref_b = Transform3D(basis_b, origin_b);
}

String JoltJointImpl3D::bodies_to_string() const {
	return vformat(
		"'%s' and '%s'",
		body_a != nullptr ? body_a->to_string() : "<unknown>",
		body_b != nullptr ? body_b->to_string() : "<World>"
	);
}
