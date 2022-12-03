#include "jolt_physics_layer_mapper_3d.hpp"

#include "utility_functions.hpp"

JoltPhysicsLayerMapper3D::JoltPhysicsLayerMapper3D() {
	mappings[GDJOLT_OBJECT_LAYER_STATIC] = JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_STATIC);
	mappings[GDJOLT_OBJECT_LAYER_MOVING] = JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_MOVING);
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)

const char* JoltPhysicsLayerMapper3D::GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const {
	switch ((JPH::BroadPhaseLayer::Type)p_layer) {
		case GDJOLT_BROAD_PHASE_LAYER_STATIC: {
			return "STATIC";
		}
		case GDJOLT_BROAD_PHASE_LAYER_MOVING: {
			return "MOVING";
		}
		default: {
			return "INVALID";
		}
	}
}

#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

bool JoltPhysicsLayerMapper3D::can_layers_collide(
	JPH::ObjectLayer p_layer1,
	JPH::ObjectLayer p_layer2
) {
	switch (p_layer1) {
		case GDJOLT_OBJECT_LAYER_STATIC: {
			return p_layer2 == GDJOLT_OBJECT_LAYER_MOVING;
		}
		case GDJOLT_OBJECT_LAYER_MOVING: {
			return true;
		}
		default: {
			ERR_FAIL_V_MSG(false, vformat("Unhandled object layer: '{}'", p_layer1));
		}
	}
}

bool JoltPhysicsLayerMapper3D::can_layers_collide(
	JPH::ObjectLayer p_layer1,
	JPH::BroadPhaseLayer p_layer2
) {
	switch (p_layer1) {
		case GDJOLT_OBJECT_LAYER_STATIC: {
			return p_layer2 == JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_MOVING);
		}
		case GDJOLT_OBJECT_LAYER_MOVING: {
			return true;
		}
		default: {
			ERR_FAIL_V_MSG(false, vformat("Unhandled object layer: '{}'", p_layer1));
		}
	}
}
