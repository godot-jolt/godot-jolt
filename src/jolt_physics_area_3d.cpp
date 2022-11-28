#include "jolt_physics_area_3d.hpp"

#include "error_macros.hpp"

Variant JoltPhysicsArea3D::get_param(PhysicsServer3D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			return gravity;
		}
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			return gravity_vector;
		}
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: {
			return linear_damp;
		}
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: {
			return angular_damp;
		}
		default: {
			ERR_FAIL_D_NOT_IMPL();
		}
	}
}

void JoltPhysicsArea3D::set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			gravity = p_value;
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			gravity_vector = p_value;
		} break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: {
			linear_damp = p_value;
		} break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: {
			angular_damp = p_value;
		} break;
		default: {
			ERR_FAIL_NOT_IMPL();
		} break;
	}
}

void JoltPhysicsArea3D::call_queries() { }
