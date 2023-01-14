#pragma once

#include "jolt_broad_phase_layer.hpp"
#include "jolt_object_layer.hpp"

class JoltLayerMapper final
	: public JPH::BroadPhaseLayerInterface
	, public JPH::ObjectLayerPairFilter
	, public JPH::ObjectVsBroadPhaseLayerFilter {
public:
	JoltLayerMapper();

	uint32_t GetNumBroadPhaseLayers() const override { return GDJOLT_BROAD_PHASE_LAYER_COUNT; }

	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer p_layer) const override {
		return mappings[p_layer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const override;
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

	bool ShouldCollide(JPH::ObjectLayer p_layer1, JPH::ObjectLayer p_layer2) const;

	bool ShouldCollide(JPH::ObjectLayer p_layer1, JPH::BroadPhaseLayer p_layer2) const;

	static JPH::ObjectLayer to_object_layer(JPH::EMotionType p_motion_type);

private:
	JPH::BroadPhaseLayer mappings[GDJOLT_OBJECT_LAYER_COUNT] = {};
};
