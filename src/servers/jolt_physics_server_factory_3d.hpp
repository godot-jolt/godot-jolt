#pragma once

#include "servers/jolt_physics_server_3d.hpp"

class JoltPhysicsServerFactory3D final : public Object {
	GDCLASS_NO_WARN(JoltPhysicsServerFactory3D, Object)

private:
	// NOLINTNEXTLINE(readability-identifier-naming)
	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("create_server"), &JoltPhysicsServerFactory3D::create_server);
	}

public:
	JoltPhysicsServer3D* create_server() { return memnew(JoltPhysicsServer3D); }
};
