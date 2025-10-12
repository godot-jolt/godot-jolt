#pragma once

class JoltProjectSettings {
public:
	static void register_settings();

	static bool is_sleep_enabled();

	static float get_sleep_velocity_threshold();

	static float get_sleep_time_threshold();

	static bool use_shape_margins();

	static bool report_all_kinematic_contacts();

	static bool use_edge_removal_for_bodies();

	static float get_soft_body_point_margin();

	static bool use_joint_world_node_a();

	static float get_ccd_movement_threshold();

	static float get_ccd_max_penetration();

	static bool use_edge_removal_for_kinematics();

	static int32_t get_kinematic_recovery_iterations();

	static float get_kinematic_recovery_amount();

	static bool use_edge_removal_for_queries();

	static bool use_legacy_ray_casting();

	static bool enable_ray_cast_face_index();

	static int32_t get_velocity_iterations();

	static int32_t get_position_iterations();

	static float get_position_correction();

	static float get_active_edge_threshold();

	static float get_bounce_velocity_threshold();

	static float get_contact_distance();

	static float get_contact_penetration();

	static bool is_pair_cache_enabled();

	static float get_pair_cache_distance();

	static float get_pair_cache_angle();

	static float get_world_boundary_shape_size();

	static float get_max_linear_velocity();

	static float get_max_angular_velocity();

	static int32_t get_max_bodies();

	static int32_t get_max_pairs();

	static int32_t get_max_contact_constraints();

	static int32_t get_max_temp_memory_mib();

	static int64_t get_max_temp_memory_b();

	static bool should_run_on_separate_thread();

	static int32_t get_max_threads();
};
