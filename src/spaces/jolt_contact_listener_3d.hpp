#pragma once

class JoltObjectImpl3D;
class JoltSpace3D;

class JoltContactListener3D final : public JPH::ContactListener {
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

		JPH::Vec3 velocity_self = {};

		JPH::Vec3 velocity_other = {};

		JPH::Vec3 impulse = {};
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
	explicit JoltContactListener3D(JoltSpace3D* p_space)
		: space(p_space) { }

	void listen_for(JoltObjectImpl3D* p_object);

	void pre_step();

	void post_step();

#ifdef GDJ_CONFIG_EDITOR
	const PackedVector3Array& get_debug_contacts() const { return debug_contacts; }

	int32_t get_debug_contact_count() const { return debug_contact_count; }

	int32_t get_max_debug_contacts() const { return (int32_t)debug_contacts.size(); }

	void set_max_debug_contacts(int32_t p_count) { debug_contacts.resize(p_count); }
#endif // GDJ_CONFIG_EDITOR

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

	bool is_listening_for(const JPH::Body& p_body) const;

	bool try_override_collision_response(
		const JPH::Body& p_body1,
		const JPH::Body& p_body2,
		JPH::ContactSettings& p_settings
	);

	bool try_apply_surface_velocities(
		const JPH::Body& p_jolt_body1,
		const JPH::Body& p_jolt_body2,
		JPH::ContactSettings& p_settings
	);

	bool try_add_contacts(
		const JPH::Body& p_body1,
		const JPH::Body& p_body2,
		const JPH::ContactManifold& p_manifold,
		JPH::ContactSettings& p_settings
	);

	bool try_add_area_overlap(
		const JPH::Body& p_body1,
		const JPH::Body& p_body2,
		const JPH::ContactManifold& p_manifold
	);

	bool try_remove_contacts(const JPH::SubShapeIDPair& p_shape_pair);

	bool try_remove_area_overlap(const JPH::SubShapeIDPair& p_shape_pair);

#ifdef GDJ_CONFIG_EDITOR
	bool try_add_debug_contacts(const JPH::ContactManifold& p_manifold);
#endif // GDJ_CONFIG_EDITOR

	void flush_contacts();

	void flush_area_enters();

	void flush_area_shifts();

	void flush_area_exits();

	ManifoldsByShapePair manifolds_by_shape_pair;

	BodyIDs listening_for;

	Overlaps area_overlaps;

	Overlaps area_enters;

	Overlaps area_exits;

	Mutex write_mutex;

	JoltSpace3D* space = nullptr;

#ifdef GDJ_CONFIG_EDITOR
	PackedVector3Array debug_contacts;

	std::atomic<int32_t> debug_contact_count = 0;
#endif // GDJ_CONFIG_EDITOR
};
