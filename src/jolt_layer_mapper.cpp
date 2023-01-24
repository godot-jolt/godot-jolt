#include "jolt_layer_mapper.hpp"

namespace {

constexpr JPH::ObjectLayer encode_layers(
	JPH::BroadPhaseLayer p_broad_phase_layer,
	JPH::ObjectLayer p_object_layer
) {
	const auto upper_bits = uint16_t((uint8_t)p_broad_phase_layer << 13U);
	const auto lower_bits = uint16_t(p_object_layer);
	return JPH::ObjectLayer(upper_bits | lower_bits);
}

constexpr JPH::ObjectLayer encode_layers(
	JoltBroadPhaseLayer p_broad_phase_layer,
	JPH::ObjectLayer p_object_layer
) {
	return encode_layers(JPH::BroadPhaseLayer(p_broad_phase_layer), p_object_layer);
}

constexpr void decode_layers(
	JPH::ObjectLayer p_encoded_layers,
	JPH::BroadPhaseLayer& p_broad_phase_layer,
	JPH::ObjectLayer& p_object_layer
) {
	p_broad_phase_layer = JPH::BroadPhaseLayer(uint8_t(p_encoded_layers >> 13U));
	p_object_layer = JPH::ObjectLayer(p_encoded_layers & 0b0001'1111'1111'1111U);
}

constexpr uint64_t encode_collision(uint32_t p_collision_layer, uint32_t p_collision_mask) {
	const auto upper_bits = (uint64_t)p_collision_layer << 32U;
	const auto lower_bits = (uint64_t)p_collision_mask;
	return upper_bits | lower_bits;
}

constexpr void decode_collision(
	uint64_t p_collision,
	uint32_t& p_collision_layer,
	uint32_t& p_collision_mask
) {
	p_collision_layer = uint32_t(p_collision >> 32U);
	p_collision_mask = uint32_t(p_collision & 0xFFFFFFFFU);
}

} // namespace

JPH::ObjectLayer JoltLayerMapper::to_object_layer(
	JoltBroadPhaseLayer p_broad_phase_layer,
	uint32_t p_collision_layer,
	uint32_t p_collision_mask
) {
	const uint64_t collision = encode_collision(p_collision_layer, p_collision_mask);

	JPH::ObjectLayer object_layer = 0;

	auto iter = layers_by_collision.find(collision);
	if (iter != layers_by_collision.end()) {
		object_layer = iter->second;
	} else {
		constexpr uint16_t object_layer_count = 1U << 13U;

		ERR_FAIL_COND_D_MSG(
			next_object_layer == object_layer_count,
			vformat("Maximum number of object layers (%d) reached.", object_layer_count)
		);

		object_layer = next_object_layer++;

		layers_by_collision[collision] = object_layer;

		{
			const MutexLockWrite collisions_lock(collisions_mutex);
			collisions_by_layer[object_layer] = collision;
		}
	}

	return encode_layers(p_broad_phase_layer, object_layer);
}

JPH::ObjectLayer JoltLayerMapper::to_object_layer(
	JPH::EMotionType p_motion_type,
	uint32_t p_collision_layer,
	uint32_t p_collision_mask
) {
	switch (p_motion_type) {
		case JPH::EMotionType::Static: {
			return to_object_layer(
				GDJOLT_BROAD_PHASE_LAYER_STATIC,
				p_collision_layer,
				p_collision_mask
			);
		}
		case JPH::EMotionType::Kinematic:
		case JPH::EMotionType::Dynamic: {
			return to_object_layer(
				GDJOLT_BROAD_PHASE_LAYER_MOVING,
				p_collision_layer,
				p_collision_mask
			);
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled motion type: '%d'", (int32_t)p_motion_type));
		}
	}
}

JPH::BroadPhaseLayer JoltLayerMapper::GetBroadPhaseLayer(JPH::ObjectLayer p_layer) const {
	JPH::BroadPhaseLayer broad_phase_layer = {};
	JPH::ObjectLayer object_layer = 0;
	decode_layers(p_layer, broad_phase_layer, object_layer);

	return broad_phase_layer;
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)

const char* JoltLayerMapper::GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const {
	switch ((JPH::BroadPhaseLayer::Type)p_layer) {
		case GDJOLT_BROAD_PHASE_LAYER_NONE: {
			return "NONE";
		}
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

bool JoltLayerMapper::ShouldCollide(JPH::ObjectLayer p_layer1, JPH::ObjectLayer p_layer2) const {
	JPH::BroadPhaseLayer broad_phase_layer1 = {};
	JPH::ObjectLayer object_layer1 = 0;
	decode_layers(p_layer1, broad_phase_layer1, object_layer1);

	JPH::BroadPhaseLayer broad_phase_layer2 = {};
	JPH::ObjectLayer object_layer2 = 0;
	decode_layers(p_layer2, broad_phase_layer2, object_layer2);

	uint64_t collision1 = 0;
	uint64_t collision2 = 0;

	{
		const MutexLockRead collisions_lock(collisions_mutex);
		collision1 = collisions_by_layer[object_layer1];
		collision2 = collisions_by_layer[object_layer2];
	}

	uint32_t collision_layer1 = 0;
	uint32_t collision_mask1 = 0;
	decode_collision(collision1, collision_layer1, collision_mask1);

	uint32_t collision_layer2 = 0;
	uint32_t collision_mask2 = 0;
	decode_collision(collision2, collision_layer2, collision_mask2);

	const bool first_scans_second = (collision_mask1 & collision_layer2) != 0;
	const bool second_scans_first = (collision_mask2 & collision_layer1) != 0;

	return first_scans_second || second_scans_first;
}

bool JoltLayerMapper::ShouldCollide(JPH::ObjectLayer p_layer1, JPH::BroadPhaseLayer p_layer2)
	const {
	JPH::BroadPhaseLayer broad_phase_layer1 = {};
	JPH::ObjectLayer object_layer1 = 0;
	decode_layers(p_layer1, broad_phase_layer1, object_layer1);

	return ShouldCollide(broad_phase_layer1, p_layer2);
}

bool JoltLayerMapper::ShouldCollide(JPH::BroadPhaseLayer p_layer1, JPH::BroadPhaseLayer p_layer2)
	const {
	const auto layer1 = (JPH::BroadPhaseLayer::Type)p_layer1;
	const auto layer2 = (JPH::BroadPhaseLayer::Type)p_layer2;

	switch (layer1) {
		case GDJOLT_BROAD_PHASE_LAYER_NONE: {
			return false;
		}
		case GDJOLT_BROAD_PHASE_LAYER_STATIC: {
			return layer2 == GDJOLT_BROAD_PHASE_LAYER_MOVING;
		}
		case GDJOLT_BROAD_PHASE_LAYER_MOVING: {
			return layer2 != GDJOLT_BROAD_PHASE_LAYER_NONE;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled object layer: '%d'", layer1));
		}
	}
}

static_assert(sizeof(JPH::ObjectLayer) == 2);
static_assert(sizeof(JPH::BroadPhaseLayer::Type) == 1);
