#include "jolt_area_3d.hpp"

#include "objects/jolt_body_3d.hpp"
#include "spaces/jolt_broad_phase_layer.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double GDJOLT_WIND_FORCE_MAGNITUDE = 0.0;
constexpr double GDJOLT_WIND_ATTENUATION_FACTOR = 0.0;

const Vector3 GDJOLT_WIND_SOURCE = {};
const Vector3 GDJOLT_WIND_DIRECTION = {};

} // namespace

Variant JoltArea3D::get_param(PhysicsServer3D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY_OVERRIDE_MODE: {
			return get_gravity_mode();
		}
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			return get_gravity();
		}
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			return get_gravity_vector();
		}
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT: {
			return is_point_gravity();
		}
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: {
			return get_point_gravity_distance();
		}
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE: {
			return get_linear_damp_mode();
		}
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: {
			return get_linear_damp();
		}
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE: {
			return get_angular_damp_mode();
		}
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: {
			return get_angular_damp();
		}
		case PhysicsServer3D::AREA_PARAM_PRIORITY: {
			return get_priority();
		}
		case PhysicsServer3D::AREA_PARAM_WIND_FORCE_MAGNITUDE: {
			return GDJOLT_WIND_FORCE_MAGNITUDE;
		}
		case PhysicsServer3D::AREA_PARAM_WIND_SOURCE: {
			return GDJOLT_WIND_SOURCE;
		}
		case PhysicsServer3D::AREA_PARAM_WIND_DIRECTION: {
			return GDJOLT_WIND_DIRECTION;
		}
		case PhysicsServer3D::AREA_PARAM_WIND_ATTENUATION_FACTOR: {
			return GDJOLT_WIND_ATTENUATION_FACTOR;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled area parameter: '%d'", p_param));
		}
	}
}

void JoltArea3D::set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY_OVERRIDE_MODE: {
			set_gravity_mode((OverrideMode)(int32_t)p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			set_gravity(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			set_gravity_vector(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT: {
			set_point_gravity(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: {
			set_point_gravity_distance(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE: {
			set_linear_damp_mode((OverrideMode)(int32_t)p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: {
			set_area_linear_damp(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE: {
			set_angular_damp_mode((OverrideMode)(int32_t)p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: {
			set_area_angular_damp(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_PRIORITY: {
			set_priority(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_WIND_FORCE_MAGNITUDE: {
			if (!Math::is_equal_approx((double)p_value, GDJOLT_WIND_FORCE_MAGNITUDE)) {
				WARN_PRINT(
					"Area wind force magnitude is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::AREA_PARAM_WIND_SOURCE: {
			if (!((Vector3)p_value).is_equal_approx(GDJOLT_WIND_SOURCE)) {
				WARN_PRINT(
					"Area wind source is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::AREA_PARAM_WIND_DIRECTION: {
			if (!((Vector3)p_value).is_equal_approx(GDJOLT_WIND_DIRECTION)) {
				WARN_PRINT(
					"Area wind direction is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		case PhysicsServer3D::AREA_PARAM_WIND_ATTENUATION_FACTOR: {
			if (!Math::is_equal_approx((double)p_value, GDJOLT_WIND_ATTENUATION_FACTOR)) {
				WARN_PRINT(
					"Area wind attenuation is not supported by Godot Jolt. "
					"Any such value will be ignored."
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled area parameter: '%d'", p_param));
		} break;
	}
}

JPH::BroadPhaseLayer JoltArea3D::get_broad_phase_layer() const {
	return monitorable
		? JoltBroadPhaseLayer::AREA_DETECTABLE
		: JoltBroadPhaseLayer::AREA_UNDETECTABLE;
}

void JoltArea3D::set_body_monitor_callback(const Callable& p_callback) {
	if (p_callback == body_monitor_callback) {
		return;
	}

	body_monitor_callback = p_callback;

	body_monitoring_changed();
}

void JoltArea3D::set_area_monitor_callback(const Callable& p_callback) {
	if (p_callback == area_monitor_callback) {
		return;
	}

	area_monitor_callback = p_callback;

	area_monitoring_changed();
}

void JoltArea3D::set_monitorable(bool p_monitorable, bool p_lock) {
	if (p_monitorable == monitorable) {
		return;
	}

	monitorable = p_monitorable;

	monitorable_changed(p_lock);
}

Vector3 JoltArea3D::compute_gravity(const Vector3& p_position, bool p_lock) {
	if (!point_gravity) {
		return gravity_vector * gravity;
	}

	const Vector3 point = get_transform_scaled(p_lock).xform(gravity_vector);
	const Vector3 to_point = point - p_position;
	const float to_point_dist_sq = max(to_point.length_squared(), CMP_EPSILON);
	const Vector3 to_point_dir = to_point / Math::sqrt(to_point_dist_sq);

	const float gravity_dist_sq = point_gravity_distance * point_gravity_distance;

	return to_point_dir * (gravity * gravity_dist_sq / to_point_dist_sq);
}

void JoltArea3D::body_shape_entered(
	const JPH::BodyID& p_body_id,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	Overlap& overlap = bodies_by_id[p_body_id];

	if (overlap.shape_pairs.size() == 0) {
		notify_body_entered(p_body_id, false);
	}

	add_shape_pair(overlap, p_body_id, p_other_shape_id, p_self_shape_id);
}

bool JoltArea3D::body_shape_exited(
	const JPH::BodyID& p_body_id,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	Overlap* overlap = bodies_by_id.getptr(p_body_id);

	if (overlap == nullptr) {
		return false;
	}

	if (!remove_shape_pair(*overlap, p_other_shape_id, p_self_shape_id)) {
		return false;
	}

	if (overlap->shape_pairs.size() == 0) {
		notify_body_exited(p_body_id, false);
	}

	return true;
}

void JoltArea3D::area_shape_entered(
	const JPH::BodyID& p_body_id,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	add_shape_pair(areas_by_id[p_body_id], p_body_id, p_other_shape_id, p_self_shape_id);
}

bool JoltArea3D::area_shape_exited(
	const JPH::BodyID& p_body_id,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	Overlap* overlap = areas_by_id.getptr(p_body_id);

	if (overlap == nullptr) {
		return false;
	}

	return remove_shape_pair(*overlap, p_other_shape_id, p_self_shape_id);
}

bool JoltArea3D::shape_exited(
	const JPH::BodyID& p_body_id,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	return body_shape_exited(p_body_id, p_other_shape_id, p_self_shape_id) ||
		area_shape_exited(p_body_id, p_other_shape_id, p_self_shape_id);
}

void JoltArea3D::call_queries() {
	flush_events(bodies_by_id, body_monitor_callback);
	flush_events(areas_by_id, area_monitor_callback);
}

void JoltArea3D::space_changing(bool p_lock) {
	if (space != nullptr) {
		// HACK(mihe): Ideally we would rely on our contact listener to report all the exits when we
		// move between (or out of) spaces, but because our Jolt body is going to be destroyed when
		// we leave this space the contact listener won't be able to retrieve the corresponding area
		// and as such cannot report any exits, so we're forced to do it manually instead.
		force_bodies_exited(true, p_lock);
		force_areas_exited(true, p_lock);
	}
}

void JoltArea3D::body_monitoring_changed() {
	if (has_body_monitor_callback()) {
		force_bodies_entered();
	} else {
		force_bodies_exited(false);
	}
}

void JoltArea3D::area_monitoring_changed() {
	if (has_area_monitor_callback()) {
		force_areas_entered();
	} else {
		force_areas_exited(false);
	}
}

void JoltArea3D::monitorable_changed(bool p_lock) {
	update_object_layer(p_lock);
}

void JoltArea3D::force_bodies_entered() {
	for (auto& [id, body] : bodies_by_id) {
		for (const auto& [id_pair, index_pair] : body.shape_pairs) {
			body.pending_added.push_back(index_pair);
		}
	}
}

void JoltArea3D::force_bodies_exited(bool p_remove, bool p_lock) {
	for (auto& [id, body] : bodies_by_id) {
		for (const auto& [id_pair, index_pair] : body.shape_pairs) {
			body.pending_removed.push_back(index_pair);
		}

		if (p_remove) {
			body.shape_pairs.clear();
			notify_body_exited(id, p_lock);
		}
	}
}

void JoltArea3D::force_areas_entered() {
	for (auto& [id, area] : areas_by_id) {
		for (const auto& [id_pair, index_pair] : area.shape_pairs) {
			area.pending_added.push_back(index_pair);
		}
	}
}

void JoltArea3D::force_areas_exited(bool p_remove, [[maybe_unused]] bool p_lock) {
	for (auto& [id, area] : areas_by_id) {
		for (const auto& [id_pair, index_pair] : area.shape_pairs) {
			area.pending_removed.push_back(index_pair);
		}

		if (p_remove) {
			area.shape_pairs.clear();
		}
	}
}

void JoltArea3D::add_shape_pair(
	Overlap& p_overlap,
	const JPH::BodyID& p_body_id,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	const JoltReadableBody3D other_jolt_body = space->read_body(p_body_id, false);
	const JoltCollisionObject3D* other_object = other_jolt_body.as_object();
	ERR_FAIL_NULL(other_object);

	p_overlap.rid = other_object->get_rid();
	p_overlap.instance_id = other_object->get_instance_id();

	ShapeIndexPair& shape_indices = p_overlap.shape_pairs[{p_other_shape_id, p_self_shape_id}];

	shape_indices.other = other_object->find_shape_index(p_other_shape_id);
	shape_indices.self = find_shape_index(p_self_shape_id);

	p_overlap.pending_added.push_back(shape_indices);
}

bool JoltArea3D::remove_shape_pair(
	Overlap& p_overlap,
	const JPH::SubShapeID& p_other_shape_id,
	const JPH::SubShapeID& p_self_shape_id
) {
	auto shape_pair = p_overlap.shape_pairs.find({p_other_shape_id, p_self_shape_id});

	if (shape_pair == p_overlap.shape_pairs.end()) {
		return false;
	}

	p_overlap.pending_removed.push_back(shape_pair->second);
	p_overlap.shape_pairs.remove(shape_pair);

	return true;
}

void JoltArea3D::flush_events(OverlapsById& p_objects, const Callable& p_callback) {
	p_objects.erase_if([&](auto& p_pair) {
		auto& [id, overlap] = p_pair;

		if (p_callback.is_valid()) {
			for (auto& shape_indices : overlap.pending_removed) {
				report_event(
					p_callback,
					PhysicsServer3D::AREA_BODY_REMOVED,
					overlap.rid,
					overlap.instance_id,
					shape_indices.other,
					shape_indices.self
				);
			}

			for (auto& shape_indices : overlap.pending_added) {
				report_event(
					p_callback,
					PhysicsServer3D::AREA_BODY_ADDED,
					overlap.rid,
					overlap.instance_id,
					shape_indices.other,
					shape_indices.self
				);
			}
		}

		overlap.pending_removed.clear();
		overlap.pending_added.clear();

		return overlap.shape_pairs.size() == 0;
	});
}

void JoltArea3D::report_event(
	const Callable& p_callback,
	PhysicsServer3D::AreaBodyStatus p_status,
	const RID& p_other_rid,
	ObjectID p_other_instance_id,
	int32_t p_other_shape_index,
	int32_t p_self_shape_index
) const {
	ERR_FAIL_COND(!p_callback.is_valid());

	static thread_local Array arguments = []() {
		Array array;
		array.resize(5);
		return array;
	}();

	arguments[0] = p_status;
	arguments[1] = p_other_rid;
	arguments[2] = p_other_instance_id;
	arguments[3] = p_other_shape_index;
	arguments[4] = p_self_shape_index;

	p_callback.callv(arguments);
}

void JoltArea3D::notify_body_entered(const JPH::BodyID& p_body_id, bool p_lock) {
	const JoltReadableBody3D jolt_body = space->read_body(p_body_id, p_lock);

	JoltBody3D* body = jolt_body.as_body();
	QUIET_FAIL_NULL(body);

	body->add_area(this, false);
}

void JoltArea3D::notify_body_exited(const JPH::BodyID& p_body_id, bool p_lock) {
	const JoltReadableBody3D jolt_body = space->read_body(p_body_id, p_lock);

	JoltBody3D* body = jolt_body.as_body();
	QUIET_FAIL_NULL(body);

	body->remove_area(this, false);
}

void JoltArea3D::create_in_space() {
	create_begin();

	jolt_settings->mIsSensor = true;
	jolt_settings->mUseManifoldReduction = false;

	create_end();
}
