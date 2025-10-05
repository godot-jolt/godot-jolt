#include "jolt_body_activation_listener_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"

void JoltBodyActivationListener3D::OnBodyActivated(
	[[maybe_unused]] const JPH::BodyID& p_body_id,
	JPH::uint64 p_body_user_data
) {
	// This method will be called on multiple threads during the simulation step.

	if (JoltBodyImpl3D* body = reinterpret_cast<JoltObjectImpl3D*>(p_body_user_data)->as_body()) {
		body->_enqueue_state_synchronization();
	}
}
