#include "jolt_layer_mapper.hpp"

#include "utility_functions.hpp"

JoltLayerMapper::JoltLayerMapper() {
	mappings[GDJOLT_OBJECT_LAYER_STATIC] = JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_STATIC);
	mappings[GDJOLT_OBJECT_LAYER_MOVING] = JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_MOVING);
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)

const char* JoltLayerMapper::GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const {
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

bool JoltLayerMapper::can_layers_collide(JPH::ObjectLayer p_layer1, JPH::ObjectLayer p_layer2) {
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

bool JoltLayerMapper::can_layers_collide(JPH::ObjectLayer p_layer1, JPH::BroadPhaseLayer p_layer2) {
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
