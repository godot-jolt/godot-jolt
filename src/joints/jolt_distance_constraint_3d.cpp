#include "jolt_distance_constraint_3d.hpp"

#include "servers/jolt_physics_server_3d.hpp"

namespace {
using ServerParamJolt = JoltPhysicsServer3D::DistanceConstraintParamJolt;
} // namespace

void JoltDistanceConstraint3D::_bind_methods() {
	BIND_METHOD(JoltDistanceConstraint3D, get_limit_spring_frequency);
	BIND_METHOD(JoltDistanceConstraint3D, set_limit_spring_frequency, "value");

	BIND_METHOD(JoltDistanceConstraint3D, get_limit_spring_damping);
	BIND_METHOD(JoltDistanceConstraint3D, set_limit_spring_damping, "value");

	BIND_METHOD(JoltDistanceConstraint3D, get_distance_min);
	BIND_METHOD(JoltDistanceConstraint3D, set_distance_min, "value");

	BIND_METHOD(JoltDistanceConstraint3D, get_distance_max);
	BIND_METHOD(JoltDistanceConstraint3D, set_distance_max, "value");

	BIND_METHOD(JoltDistanceConstraint3D, get_point_a);
	BIND_METHOD(JoltDistanceConstraint3D, set_point_a, "value");

	BIND_METHOD(JoltDistanceConstraint3D, get_global_point_a);

	BIND_METHOD(JoltDistanceConstraint3D, get_point_b);
	BIND_METHOD(JoltDistanceConstraint3D, set_point_b, "value");

	BIND_METHOD(JoltDistanceConstraint3D, get_global_point_b);

	ADD_GROUP("Distance", "distance_");

	BIND_PROPERTY("distance_min", Variant::FLOAT, "suffix:m");
	BIND_PROPERTY("distance_max", Variant::FLOAT, "suffix:m");

	ADD_GROUP("Limit Spring", "limit_spring_");

	BIND_PROPERTY("limit_spring_frequency", Variant::FLOAT, "suffix:hz");
	BIND_PROPERTY("limit_spring_damping", Variant::FLOAT);

	ADD_GROUP("Anchor", "point_");

	BIND_PROPERTY("point_a", Variant::VECTOR3, "m");
	BIND_PROPERTY("point_b", Variant::VECTOR3, "m");
}

void JoltDistanceConstraint3D::set_limit_spring_frequency(double p_value) {
	if (limit_spring_frequency == p_value) {
		return;
	}

	limit_spring_frequency = p_value;

	_update_jolt_param(PARAM_LIMITS_SPRING_FREQUENCY);
}

void JoltDistanceConstraint3D::set_limit_spring_damping(double p_value) {
	if (limit_spring_damping == p_value) {
		return;
	}

	limit_spring_damping = p_value;

	_update_jolt_param(PARAM_LIMITS_SPRING_DAMPING);
}

void JoltDistanceConstraint3D::set_distance_min(double p_value) {
	if (distance_min == p_value) {
		return;
	}

	distance_min = p_value;

	_update_jolt_param(PARAM_DISTANCE_MIN);
}

void JoltDistanceConstraint3D::set_distance_max(double p_value) {
	if (distance_max == p_value) {
		return;
	}

	distance_max = p_value;

	_update_jolt_param(PARAM_DISTANCE_MAX);
}

void JoltDistanceConstraint3D::set_point_a(Vector3 p_point) {
	if (point_a == p_point) {
		return;
	}

	_points_changing();
	point_a = p_point;
	_points_changed();
}

void JoltDistanceConstraint3D::set_point_b(Vector3 p_point) {
	if (point_b == p_point) {
		return;
	}

	_points_changing();
	point_b = p_point;
	_points_changed();
}

Vector3 JoltDistanceConstraint3D::get_global_point_a() const {
	PhysicsBody3D* body_a = get_body_a();
	if (body_a == nullptr) {
		return to_global(point_a);
	}
	return body_a->to_global(point_a);
}

Vector3 JoltDistanceConstraint3D::get_global_point_b() const {
	PhysicsBody3D* body_b = get_body_b();
	if (body_b == nullptr) {
		return to_global(point_b);
	}
	return body_b->to_global(point_b);
}

void JoltDistanceConstraint3D::_configure(PhysicsBody3D* p_body_a, PhysicsBody3D* p_body_b) {
	JoltPhysicsServer3D* physics_server = _get_jolt_physics_server();
	ERR_FAIL_NULL(physics_server);

	const bool are_bodies_switched = get_body_a() == nullptr; // Bodies get switched in
															  // JoltJoint3D::_configure.
	const Vector3 global_position = are_bodies_switched
		? get_global_point_a()
		: get_global_point_b();
	const Vector3 p_body_a_point = are_bodies_switched ? point_b : point_a;
	const Vector3 p_body_b_point = are_bodies_switched ? point_a : point_b;

	physics_server->joint_make_distance_constraint(
		rid,
		p_body_a->get_rid(),
		p_body_a_point,
		p_body_b != nullptr ? p_body_b->get_rid() : RID(),
		p_body_b != nullptr ? p_body_b_point : global_position
	);

	_update_jolt_param(PARAM_LIMITS_SPRING_FREQUENCY);
	_update_jolt_param(PARAM_LIMITS_SPRING_DAMPING);
	_update_jolt_param(PARAM_DISTANCE_MIN);
	_update_jolt_param(PARAM_DISTANCE_MAX);
}

void JoltDistanceConstraint3D::_update_jolt_param(Param p_param) {
	QUIET_FAIL_COND(_is_invalid());

	JoltPhysicsServer3D* physics_server = _get_jolt_physics_server();
	QUIET_FAIL_NULL(physics_server);

	double* value = nullptr;

	switch (p_param) {
		case PARAM_LIMITS_SPRING_FREQUENCY: {
			value = &limit_spring_frequency;
		} break;
		case PARAM_LIMITS_SPRING_DAMPING: {
			value = &limit_spring_damping;
		} break;
		case PARAM_DISTANCE_MIN: {
			value = &distance_min;
		} break;
		case PARAM_DISTANCE_MAX: {
			value = &distance_max;
		} break;
		default: {
			ERR_FAIL_REPORT(vformat("Unhandled parameter: `%d`.", p_param));
		} break;
	}

	physics_server->distance_constraint_set_jolt_param(rid, ServerParamJolt(p_param), *value);
}

void JoltDistanceConstraint3D::_points_changing() {
	_destroy();
}

void JoltDistanceConstraint3D::_points_changed() {
	_build();
}
