#pragma once

class JoltCollisionObject3D;
class JoltSpace3D;

class JoltContactListener final : public JPH::ContactListener {
	using Mutex = std::mutex;

	using MutexLock = std::unique_lock<Mutex>;

	struct BodyIDHasher {
		static uint32_t hash(const JPH::BodyID& p_id) {
			return hash_fmix32(p_id.GetIndexAndSequenceNumber());
		}
	};

	struct ShapePairHasher {
		static uint32_t hash(const JPH::SubShapeIDPair& p_pair) {
			uint32_t hash = hash_murmur3_one_32(p_pair.GetBody1ID().GetIndexAndSequenceNumber());
			hash = hash_murmur3_one_32(p_pair.GetSubShapeID1().GetValue(), hash);
			hash = hash_murmur3_one_32(p_pair.GetBody2ID().GetIndexAndSequenceNumber(), hash);
			hash = hash_murmur3_one_32(p_pair.GetSubShapeID2().GetValue(), hash);
			return hash_fmix32(hash);
		}
	};

	struct Contact {
		JPH::Vec3 normal = {};

		JPH::Vec3 point_self = {};

		JPH::Vec3 point_other = {};

		JPH::Vec3 velocity_other = {};

		float impulse = {};
	};

	using Contacts = LocalVector<Contact>;

	struct Manifold {
		Contacts contacts1;

		Contacts contacts2;

		float depth = 0.0f;
	};

	using BodyIDs = HashSet<JPH::BodyID, BodyIDHasher>;

	using Overlaps = HashSet<JPH::SubShapeIDPair, ShapePairHasher>;

	using ManifoldsByShapePair = HashMap<JPH::SubShapeIDPair, Manifold, ShapePairHasher>;

public:
	explicit JoltContactListener(JoltSpace3D* p_space)
		: space(p_space) { }

	void listen_for(JoltCollisionObject3D* p_object);

	void pre_step();

	void post_step();

private:
	void OnContactAdded(
		const JPH::Body& p_body1,
		const JPH::Body& p_body2,
		const JPH::ContactManifold& p_manifold,
		JPH::ContactSettings& p_settings
	) override;

	void OnContactPersisted(
		const JPH::Body& p_body1,
		const JPH::Body& p_body2,
		const JPH::ContactManifold& p_manifold,
		JPH::ContactSettings& p_settings
	) override;

	void OnContactRemoved(const JPH::SubShapeIDPair& p_shape_pair) override;

	bool is_listening_for(const JPH::Body& p_body);

	void update_contacts(
		const JPH::Body& p_body1,
		const JPH::Body& p_body2,
		const JPH::ContactManifold& p_manifold,
		JPH::ContactSettings& p_settings
	);

	void flush_contacts();

	void flush_area_enters();

	void flush_area_shifts();

	void flush_area_exits();

	JoltSpace3D* space = nullptr;

	Mutex write_mutex;

	BodyIDs listening_for;

	ManifoldsByShapePair manifolds_by_shape_pair;

	Overlaps area_overlaps;

	Overlaps area_enters;

	Overlaps area_exits;
};
