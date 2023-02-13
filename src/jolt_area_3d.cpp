#include "jolt_area_3d.hpp"

Variant JoltArea3D::get_param(PhysicsServer3D::AreaParameter p_param) const {
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

void JoltArea3D::set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value) {
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

void JoltArea3D::call_queries() {
	// TOOD(mihe): Call `area_monitor_callback`
}

void JoltArea3D::create_in_space(bool p_lock) {
	JPH::BodyCreationSettings settings = create_begin();

	settings.mIsSensor = true;

	create_end(settings, p_lock);
}
