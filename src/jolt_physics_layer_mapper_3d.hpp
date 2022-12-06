#pragma once

#include "jolt_physics_broad_phase_layer_3d.hpp"
#include "jolt_physics_object_layer_3d.hpp"

class JoltLayerMapper final : public JPH::BroadPhaseLayerInterface {
public:
	JoltLayerMapper();

	uint32_t GetNumBroadPhaseLayers() const override { return GDJOLT_BROAD_PHASE_LAYER_COUNT; }

	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer p_layer) const override {
		return mappings[p_layer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const override;
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

	static bool can_layers_collide(JPH::ObjectLayer p_layer1, JPH::ObjectLayer p_layer2);

	static bool can_layers_collide(JPH::ObjectLayer p_layer1, JPH::BroadPhaseLayer p_layer2);

private:
	JPH::BroadPhaseLayer mappings[GDJOLT_OBJECT_LAYER_COUNT] = {};
};
