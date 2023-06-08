#pragma once

#include "objects/jolt_object_impl_3d.hpp"

class JoltAreaImpl3D final : public JoltObjectImpl3D {
	struct BodyIDHasher {
		static uint32_t hash(const JPH::BodyID& p_id) {
			return hash_fmix32(p_id.GetIndexAndSequenceNumber());
		}
	};

	struct ShapeIDPair {
		ShapeIDPair(JPH::SubShapeID p_other, JPH::SubShapeID p_self)
			: other(p_other)
			, self(p_self) { }

		static uint32_t hash(const ShapeIDPair& p_pair) {
			uint32_t hash = hash_murmur3_one_32(p_pair.other.GetValue());
			hash = hash_murmur3_one_32(p_pair.self.GetValue(), hash);
			return hash_fmix32(hash);
		}

		friend bool operator==(const ShapeIDPair& p_lhs, const ShapeIDPair& p_rhs) {
			return std::tie(p_lhs.other, p_lhs.self) == std::tie(p_rhs.other, p_rhs.self);
		}

		JPH::SubShapeID other;

		JPH::SubShapeID self;
	};

	struct ShapeIndexPair {
		int32_t other = -1;

		int32_t self = -1;
	};

	struct Overlap {
		HashMap<ShapeIDPair, ShapeIndexPair, ShapeIDPair> shape_pairs;

		InlineVector<ShapeIndexPair, 1> pending_added;

		InlineVector<ShapeIndexPair, 1> pending_removed;

		RID rid;

		ObjectID instance_id;
	};

	using OverlapsById = HashMap<JPH::BodyID, Overlap, BodyIDHasher>;

public:
	using OverrideMode = PhysicsServer3D::AreaSpaceOverrideMode;

	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value);

	JPH::BroadPhaseLayer get_broad_phase_layer() const override;

	bool has_body_monitor_callback() const { return body_monitor_callback.is_valid(); }

	void set_body_monitor_callback(const Callable& p_callback);

	bool has_area_monitor_callback() const { return area_monitor_callback.is_valid(); }

	void set_area_monitor_callback(const Callable& p_callback);

	bool is_monitoring() const {
		return has_body_monitor_callback() || has_area_monitor_callback();
	}

	bool is_monitorable() const { return monitorable; }

	void set_monitorable(bool p_monitorable, bool p_lock = true);

	bool generates_contacts() const override { return is_monitoring(); }

	bool is_point_gravity() const { return point_gravity; }

	void set_point_gravity(bool p_enabled) { point_gravity = p_enabled; }

	float get_priority() const { return priority; }

	void set_priority(float p_priority) { priority = p_priority; }

	float get_gravity() const { return gravity; }

	void set_gravity(float p_gravity) { gravity = p_gravity; }

	float get_point_gravity_distance() const { return point_gravity_distance; }

	void set_point_gravity_distance(float p_distance) { point_gravity_distance = p_distance; }

	float get_linear_damp() const { return linear_damp; }

	void set_area_linear_damp(float p_damp) { linear_damp = p_damp; }

	float get_angular_damp() const { return angular_damp; }

	void set_area_angular_damp(float p_damp) { angular_damp = p_damp; }

	OverrideMode get_gravity_mode() const { return gravity_mode; }

	void set_gravity_mode(OverrideMode p_mode) { gravity_mode = p_mode; }

	OverrideMode get_linear_damp_mode() const { return linear_damp_mode; }

	void set_linear_damp_mode(OverrideMode p_mode) { linear_damp_mode = p_mode; }

	OverrideMode get_angular_damp_mode() const { return angular_damp_mode; }

	void set_angular_damp_mode(OverrideMode p_mode) { angular_damp_mode = p_mode; }

	Vector3 get_gravity_vector() const { return gravity_vector; }

	void set_gravity_vector(const Vector3& p_vector) { gravity_vector = p_vector; }

	Vector3 compute_gravity(const Vector3& p_position, bool p_lock = true) const;

	void body_shape_entered(
		const JPH::BodyID& p_body_id,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	bool body_shape_exited(
		const JPH::BodyID& p_body_id,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	void area_shape_entered(
		const JPH::BodyID& p_body_id,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	bool area_shape_exited(
		const JPH::BodyID& p_body_id,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	bool shape_exited(
		const JPH::BodyID& p_body_id,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	void call_queries(JPH::Body& p_jolt_body);

	bool has_custom_center_of_mass() const override { return false; }

	Vector3 get_center_of_mass_custom() const override { return {0, 0, 0}; }

private:
	JPH::EMotionType get_motion_type() const override { return JPH::EMotionType::Kinematic; }

	void create_in_space() override;

	void add_shape_pair(
		Overlap& p_overlap,
		const JPH::BodyID& p_body_id,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	bool remove_shape_pair(
		Overlap& p_overlap,
		const JPH::SubShapeID& p_other_shape_id,
		const JPH::SubShapeID& p_self_shape_id
	);

	void flush_events(OverlapsById& p_objects, const Callable& p_callback);

	void report_event(
		const Callable& p_callback,
		PhysicsServer3D::AreaBodyStatus p_status,
		const RID& p_other_rid,
		ObjectID p_other_instance_id,
		int32_t p_other_shape_index,
		int32_t p_self_shape_index
	) const;

	void notify_body_entered(const JPH::BodyID& p_body_id, bool p_lock = true);

	void notify_body_exited(const JPH::BodyID& p_body_id, bool p_lock = true);

	void force_bodies_entered();

	void force_bodies_exited(bool p_remove, bool p_lock = true);

	void force_areas_entered();

	void force_areas_exited(bool p_remove, bool p_lock = true);

	void space_changing(bool p_lock = true) override;

	void body_monitoring_changed();

	void area_monitoring_changed();

	void monitorable_changed(bool p_lock = true);

	OverlapsById bodies_by_id;

	OverlapsById areas_by_id;

	Vector3 gravity_vector = {0, -1, 0};

	Callable body_monitor_callback;

	Callable area_monitor_callback;

	float priority = 0.0f;

	float gravity = 9.8f;

	float point_gravity_distance = 0.0f;

	float linear_damp = 0.1f;

	float angular_damp = 0.1f;

	OverrideMode gravity_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;

	OverrideMode linear_damp_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;

	OverrideMode angular_damp_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;

	bool monitorable = false;

	bool point_gravity = false;
};
