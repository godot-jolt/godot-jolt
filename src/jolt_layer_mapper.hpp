#pragma once

#include "jolt_broad_phase_layer.hpp"

class JoltLayerMapper final
	: public JPH::BroadPhaseLayerInterface
	, public JPH::ObjectLayerPairFilter
	, public JPH::ObjectVsBroadPhaseLayerFilter {
	using Mutex = std::shared_mutex;
	using MutexLockRead = std::shared_lock<Mutex>;
	using MutexLockWrite = std::unique_lock<Mutex>;

public:
	JPH::ObjectLayer to_object_layer(
		JoltBroadPhaseLayer p_broad_phase_layer,
		uint32_t p_collision_layer,
		uint32_t p_collision_mask
	);

	JPH::ObjectLayer to_object_layer(
		JPH::EMotionType p_motion_type,
		uint32_t p_collision_layer,
		uint32_t p_collision_mask
	);

private:
	uint32_t GetNumBroadPhaseLayers() const override { return GDJOLT_BROAD_PHASE_LAYER_COUNT; }

	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer p_layer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const override;
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

	bool ShouldCollide(JPH::ObjectLayer p_layer1, JPH::ObjectLayer p_layer2) const override;

	bool ShouldCollide(JPH::ObjectLayer p_layer1, JPH::BroadPhaseLayer p_layer2) const override;

	bool ShouldCollide(JPH::BroadPhaseLayer p_layer1, JPH::BroadPhaseLayer p_layer2) const;

	JPH::ObjectLayer next_object_layer = 0;

	HashMap<uint64_t, JPH::ObjectLayer> layers_by_collision;

	HashMap<JPH::ObjectLayer, uint64_t> collisions_by_layer;

	mutable Mutex collisions_mutex;
};
