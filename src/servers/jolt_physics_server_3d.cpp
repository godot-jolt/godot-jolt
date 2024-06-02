#include "jolt_physics_server_3d.hpp"

#include "joints/jolt_cone_twist_joint_impl_3d.hpp"
#include "joints/jolt_generic_6dof_joint_impl_3d.hpp"
#include "joints/jolt_hinge_joint_impl_3d.hpp"
#include "joints/jolt_joint_impl_3d.hpp"
#include "joints/jolt_pin_joint_impl_3d.hpp"
#include "joints/jolt_slider_joint_impl_3d.hpp"
#include "objects/jolt_area_impl_3d.hpp"
#include "objects/jolt_body_impl_3d.hpp"
#include "objects/jolt_soft_body_impl_3d.hpp"
#include "shapes/jolt_box_shape_impl_3d.hpp"
#include "shapes/jolt_capsule_shape_impl_3d.hpp"
#include "shapes/jolt_concave_polygon_shape_impl_3d.hpp"
#include "shapes/jolt_convex_polygon_shape_impl_3d.hpp"
#include "shapes/jolt_cylinder_shape_impl_3d.hpp"
#include "shapes/jolt_height_map_shape_impl_3d.hpp"
#include "shapes/jolt_separation_ray_shape_impl_3d.hpp"
#include "shapes/jolt_sphere_shape_impl_3d.hpp"
#include "shapes/jolt_world_boundary_shape_impl_3d.hpp"
#include "spaces/jolt_job_system.hpp"
#include "spaces/jolt_physics_direct_space_state_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr char PHYSICS_SERVER_NAME[] = "JoltPhysicsServer3D";

} // namespace

void JoltPhysicsServer3D::_bind_methods() {
	// clang-format off

#ifdef GDJ_CONFIG_EDITOR
	BIND_METHOD(JoltPhysicsServer3D, dump_debug_snapshots, "dir");
	BIND_METHOD(JoltPhysicsServer3D, space_dump_debug_snapshot, "space", "dir");
#endif // GDJ_CONFIG_EDITOR

	BIND_METHOD(JoltPhysicsServer3D, joint_get_enabled, "joint");
	BIND_METHOD(JoltPhysicsServer3D, joint_set_enabled, "joint", "enabled");

	BIND_METHOD(JoltPhysicsServer3D, joint_get_solver_velocity_iterations, "joint");
	BIND_METHOD(JoltPhysicsServer3D, joint_set_solver_velocity_iterations, "joint", "value");

	BIND_METHOD(JoltPhysicsServer3D, joint_get_solver_position_iterations, "joint");
	BIND_METHOD(JoltPhysicsServer3D, joint_set_solver_position_iterations, "joint", "value");

	BIND_METHOD(JoltPhysicsServer3D, pin_joint_get_applied_force, "joint");

	BIND_METHOD(JoltPhysicsServer3D, hinge_joint_get_jolt_param, "joint", "param");
	BIND_METHOD(JoltPhysicsServer3D, hinge_joint_set_jolt_param, "joint", "param", "value");

	BIND_METHOD(JoltPhysicsServer3D, hinge_joint_get_jolt_flag, "joint", "flag");
	BIND_METHOD(JoltPhysicsServer3D, hinge_joint_set_jolt_flag, "joint", "flag", "value");

	BIND_METHOD(JoltPhysicsServer3D, hinge_joint_get_applied_force, "joint");
	BIND_METHOD(JoltPhysicsServer3D, hinge_joint_get_applied_torque, "joint");

	BIND_METHOD(JoltPhysicsServer3D, slider_joint_get_jolt_param, "joint", "param");
	BIND_METHOD(JoltPhysicsServer3D, slider_joint_set_jolt_param, "joint", "param", "value");

	BIND_METHOD(JoltPhysicsServer3D, slider_joint_get_jolt_flag, "joint", "flag");
	BIND_METHOD(JoltPhysicsServer3D, slider_joint_set_jolt_flag, "joint", "flag", "value");

	BIND_METHOD(JoltPhysicsServer3D, slider_joint_get_applied_force, "joint");
	BIND_METHOD(JoltPhysicsServer3D, slider_joint_get_applied_torque, "joint");

	BIND_METHOD(JoltPhysicsServer3D, cone_twist_joint_get_jolt_param, "joint", "param");
	BIND_METHOD(JoltPhysicsServer3D, cone_twist_joint_set_jolt_param, "joint", "param", "value");

	BIND_METHOD(JoltPhysicsServer3D, cone_twist_joint_get_jolt_flag, "joint", "flag");
	BIND_METHOD(JoltPhysicsServer3D, cone_twist_joint_set_jolt_flag, "joint", "flag", "value");

	BIND_METHOD(JoltPhysicsServer3D, cone_twist_joint_get_applied_force, "joint");
	BIND_METHOD(JoltPhysicsServer3D, cone_twist_joint_get_applied_torque, "joint");

	BIND_METHOD(JoltPhysicsServer3D, generic_6dof_joint_get_jolt_param, "joint", "axis", "param");
	BIND_METHOD(JoltPhysicsServer3D, generic_6dof_joint_set_jolt_param, "joint", "axis", "param", "value");

	BIND_METHOD(JoltPhysicsServer3D, generic_6dof_joint_get_jolt_flag, "joint", "axis", "flag");
	BIND_METHOD(JoltPhysicsServer3D, generic_6dof_joint_set_jolt_flag, "joint", "axis", "flag", "value");

	BIND_METHOD(JoltPhysicsServer3D, generic_6dof_joint_get_applied_force, "joint");
	BIND_METHOD(JoltPhysicsServer3D, generic_6dof_joint_get_applied_torque, "joint");

	// clang-format on

	BIND_ENUM_CONSTANT(HINGE_JOINT_LIMIT_SPRING_FREQUENCY);
	BIND_ENUM_CONSTANT(HINGE_JOINT_LIMIT_SPRING_DAMPING);
	BIND_ENUM_CONSTANT(HINGE_JOINT_MOTOR_MAX_TORQUE);

	BIND_ENUM_CONSTANT(HINGE_JOINT_FLAG_USE_LIMIT_SPRING);

	BIND_ENUM_CONSTANT(SLIDER_JOINT_LIMIT_SPRING_FREQUENCY);
	BIND_ENUM_CONSTANT(SLIDER_JOINT_LIMIT_SPRING_DAMPING);
	BIND_ENUM_CONSTANT(SLIDER_JOINT_MOTOR_TARGET_VELOCITY);
	BIND_ENUM_CONSTANT(SLIDER_JOINT_MOTOR_MAX_FORCE);

	BIND_ENUM_CONSTANT(SLIDER_JOINT_FLAG_USE_LIMIT);
	BIND_ENUM_CONSTANT(SLIDER_JOINT_FLAG_USE_LIMIT_SPRING);
	BIND_ENUM_CONSTANT(SLIDER_JOINT_FLAG_ENABLE_MOTOR);

	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_SWING_MOTOR_TARGET_VELOCITY_Y);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_SWING_MOTOR_TARGET_VELOCITY_Z);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_TWIST_MOTOR_TARGET_VELOCITY);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_SWING_MOTOR_MAX_TORQUE);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_TWIST_MOTOR_MAX_TORQUE);

	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_FLAG_USE_SWING_LIMIT);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_FLAG_USE_TWIST_LIMIT);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_FLAG_ENABLE_SWING_MOTOR);
	BIND_ENUM_CONSTANT(CONE_TWIST_JOINT_FLAG_ENABLE_TWIST_MOTOR);

	BIND_ENUM_CONSTANT(G6DOF_JOINT_LINEAR_SPRING_FREQUENCY);
	BIND_ENUM_CONSTANT(G6DOF_JOINT_LINEAR_LIMIT_SPRING_FREQUENCY);
	BIND_ENUM_CONSTANT(G6DOF_JOINT_LINEAR_LIMIT_SPRING_DAMPING);
	BIND_ENUM_CONSTANT(G6DOF_JOINT_ANGULAR_SPRING_FREQUENCY);

	BIND_ENUM_CONSTANT(G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT_SPRING);
	BIND_ENUM_CONSTANT(G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING_FREQUENCY);
	BIND_ENUM_CONSTANT(G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING_FREQUENCY);
}

JoltPhysicsServer3D::JoltPhysicsServer3D() {
	Engine* engine = Engine::get_singleton();

	if (engine->has_singleton(PHYSICS_SERVER_NAME)) {
		engine->unregister_singleton(PHYSICS_SERVER_NAME);
	}

	engine->register_singleton(PHYSICS_SERVER_NAME, this);
}

JoltPhysicsServer3D::~JoltPhysicsServer3D() {
	Engine::get_singleton()->unregister_singleton(PHYSICS_SERVER_NAME);
}

JoltPhysicsServer3D* JoltPhysicsServer3D::get_singleton() {
	static auto* instance = dynamic_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());
	return instance;
}

RID JoltPhysicsServer3D::_world_boundary_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltWorldBoundaryShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_separation_ray_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltSeparationRayShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_sphere_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltSphereShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_box_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltBoxShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_capsule_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltCapsuleShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_cylinder_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltCylinderShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_convex_polygon_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltConvexPolygonShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_concave_polygon_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltConcavePolygonShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_heightmap_shape_create() {
	JoltShapeImpl3D* shape = memnew(JoltHeightMapShapeImpl3D);
	RID rid = shape_owner.make_rid(shape);
	shape->set_rid(rid);
	return rid;
}

RID JoltPhysicsServer3D::_custom_shape_create() {
	ERR_FAIL_D_MSG("Custom shapes are not supported by Godot Jolt.");
}

void JoltPhysicsServer3D::_shape_set_data(const RID& p_shape, const Variant& p_data) {
	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	shape->set_data(p_data);
}

void JoltPhysicsServer3D::_shape_set_custom_solver_bias(const RID& p_shape, double p_bias) {
	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	shape->set_solver_bias((float)p_bias);
}

PhysicsServer3D::ShapeType JoltPhysicsServer3D::_shape_get_type(const RID& p_shape) const {
	const JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_D(shape);

	return shape->get_type();
}

Variant JoltPhysicsServer3D::_shape_get_data(const RID& p_shape) const {
	const JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_D(shape);

	return shape->get_data();
}

void JoltPhysicsServer3D::_shape_set_margin(const RID& p_shape, double p_margin) {
	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	shape->set_margin((float)p_margin);
}

double JoltPhysicsServer3D::_shape_get_margin(const RID& p_shape) const {
	const JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_D(shape);

	return (double)shape->get_margin();
}

double JoltPhysicsServer3D::_shape_get_custom_solver_bias(const RID& p_shape) const {
	const JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_D(shape);

	return (double)shape->get_solver_bias();
}

RID JoltPhysicsServer3D::_space_create() {
	JoltSpace3D* space = memnew(JoltSpace3D(job_system));
	RID rid = space_owner.make_rid(space);
	space->set_rid(rid);

	const RID default_area_rid = area_create();
	JoltAreaImpl3D* default_area = area_owner.get_or_null(default_area_rid);
	ERR_FAIL_NULL_D(default_area);
	space->set_default_area(default_area);
	default_area->set_space(space);

	return rid;
}

void JoltPhysicsServer3D::_space_set_active(const RID& p_space, bool p_active) {
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	if (p_active) {
		space->set_active(true);
		active_spaces.insert(space);
	} else {
		space->set_active(false);
		active_spaces.erase(space);
	}
}

bool JoltPhysicsServer3D::_space_is_active(const RID& p_space) const {
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_D(space);

	return active_spaces.has(space);
}

void JoltPhysicsServer3D::_space_set_param(
	const RID& p_space,
	SpaceParameter p_param,
	double p_value
) {
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	space->set_param(p_param, p_value);
}

double JoltPhysicsServer3D::_space_get_param(const RID& p_space, SpaceParameter p_param) const {
	const JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_D(space);

	return space->get_param(p_param);
}

PhysicsDirectSpaceState3D* JoltPhysicsServer3D::_space_get_direct_state(const RID& p_space) {
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_D(space);

	return space->get_direct_state();
}

void JoltPhysicsServer3D::_space_set_debug_contacts(
	[[maybe_unused]] const RID& p_space,
	[[maybe_unused]] int32_t p_max_contacts
) {
#ifdef GDJ_CONFIG_EDITOR
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	space->set_max_debug_contacts(p_max_contacts);
#endif // GDJ_CONFIG_EDITOR
}

PackedVector3Array JoltPhysicsServer3D::_space_get_contacts([[maybe_unused]] const RID& p_space
) const {
#ifdef GDJ_CONFIG_EDITOR
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_D(space);

	return space->get_debug_contacts();
#else // GDJ_CONFIG_EDITOR
	return {};
#endif // GDJ_CONFIG_EDITOR
}

int32_t JoltPhysicsServer3D::_space_get_contact_count([[maybe_unused]] const RID& p_space) const {
#ifdef GDJ_CONFIG_EDITOR
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_D(space);

	return space->get_debug_contact_count();
#else // GDJ_CONFIG_EDITOR
	return 0;
#endif // GDJ_CONFIG_EDITOR
}

RID JoltPhysicsServer3D::_area_create() {
	JoltAreaImpl3D* area = memnew(JoltAreaImpl3D);
	RID rid = area_owner.make_rid(area);
	area->set_rid(rid);
	return rid;
}

void JoltPhysicsServer3D::_area_set_space(const RID& p_area, const RID& p_space) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	JoltSpace3D* space = nullptr;

	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_NULL(space);
	}

	area->set_space(space);
}

RID JoltPhysicsServer3D::_area_get_space(const RID& p_area) const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	const JoltSpace3D* space = area->get_space();

	if (space == nullptr) {
		return {};
	}

	return space->get_rid();
}

void JoltPhysicsServer3D::_area_add_shape(
	const RID& p_area,
	const RID& p_shape,
	const Transform3D& p_transform,
	bool p_disabled
) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	area->add_shape(shape, p_transform, p_disabled);
}

void JoltPhysicsServer3D::_area_set_shape(
	const RID& p_area,
	int32_t p_shape_idx,
	const RID& p_shape
) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	area->set_shape(p_shape_idx, shape);
}

void JoltPhysicsServer3D::_area_set_shape_transform(
	const RID& p_area,
	int32_t p_shape_idx,
	const Transform3D& p_transform
) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_shape_transform(p_shape_idx, p_transform);
}

int32_t JoltPhysicsServer3D::_area_get_shape_count(const RID& p_area) const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	return area->get_shape_count();
}

RID JoltPhysicsServer3D::_area_get_shape(const RID& p_area, int32_t p_shape_idx) const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	const JoltShapeImpl3D* shape = area->get_shape(p_shape_idx);
	ERR_FAIL_NULL_D(shape);

	return shape->get_rid();
}

Transform3D JoltPhysicsServer3D::_area_get_shape_transform(const RID& p_area, int32_t p_shape_idx)
	const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	return area->get_shape_transform_scaled(p_shape_idx);
}

void JoltPhysicsServer3D::_area_remove_shape(const RID& p_area, int32_t p_shape_idx) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->remove_shape(p_shape_idx);
}

void JoltPhysicsServer3D::_area_clear_shapes(const RID& p_area) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->clear_shapes();
}

void JoltPhysicsServer3D::_area_set_shape_disabled(
	const RID& p_area,
	int32_t p_shape_idx,
	bool p_disabled
) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_shape_disabled(p_shape_idx, p_disabled);
}

void JoltPhysicsServer3D::_area_attach_object_instance_id(const RID& p_area, uint64_t p_id) {
	RID area_rid = p_area;

	if (space_owner.owns(area_rid)) {
		const JoltSpace3D* space = space_owner.get_or_null(area_rid);
		area_rid = space->get_default_area()->get_rid();
	}

	JoltAreaImpl3D* area = area_owner.get_or_null(area_rid);
	ERR_FAIL_NULL(area);

	area->set_instance_id(ObjectID(p_id));
}

uint64_t JoltPhysicsServer3D::_area_get_object_instance_id(const RID& p_area) const {
	RID area_rid = p_area;

	if (space_owner.owns(area_rid)) {
		const JoltSpace3D* space = space_owner.get_or_null(area_rid);
		area_rid = space->get_default_area()->get_rid();
	}

	JoltAreaImpl3D* area = area_owner.get_or_null(area_rid);
	ERR_FAIL_NULL_D(area);

	return area->get_instance_id();
}

void JoltPhysicsServer3D::_area_set_param(
	const RID& p_area,
	AreaParameter p_param,
	const Variant& p_value
) {
	RID area_rid = p_area;

	if (space_owner.owns(area_rid)) {
		const JoltSpace3D* space = space_owner.get_or_null(area_rid);
		area_rid = space->get_default_area()->get_rid();
	}

	JoltAreaImpl3D* area = area_owner.get_or_null(area_rid);
	ERR_FAIL_NULL(area);

	area->set_param(p_param, p_value);
}

void JoltPhysicsServer3D::_area_set_transform(const RID& p_area, const Transform3D& p_transform) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	return area->set_transform(p_transform);
}

Variant JoltPhysicsServer3D::_area_get_param(const RID& p_area, AreaParameter p_param) const {
	RID area_rid = p_area;

	if (space_owner.owns(area_rid)) {
		const JoltSpace3D* space = space_owner.get_or_null(area_rid);
		area_rid = space->get_default_area()->get_rid();
	}

	JoltAreaImpl3D* area = area_owner.get_or_null(area_rid);
	ERR_FAIL_NULL_D(area);

	return area->get_param(p_param);
}

Transform3D JoltPhysicsServer3D::_area_get_transform(const RID& p_area) const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	return area->get_transform_scaled();
}

void JoltPhysicsServer3D::_area_set_collision_mask(const RID& p_area, uint32_t p_mask) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_collision_mask(p_mask);
}

uint32_t JoltPhysicsServer3D::_area_get_collision_mask(const RID& p_area) const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	return area->get_collision_mask();
}

void JoltPhysicsServer3D::_area_set_collision_layer(const RID& p_area, uint32_t p_layer) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_collision_layer(p_layer);
}

uint32_t JoltPhysicsServer3D::_area_get_collision_layer(const RID& p_area) const {
	const JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_D(area);

	return area->get_collision_layer();
}

void JoltPhysicsServer3D::_area_set_monitorable(const RID& p_area, bool p_monitorable) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_monitorable(p_monitorable);
}

void JoltPhysicsServer3D::_area_set_monitor_callback(
	const RID& p_area,
	const Callable& p_callback
) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_body_monitor_callback(p_callback);
}

void JoltPhysicsServer3D::_area_set_area_monitor_callback(
	const RID& p_area,
	const Callable& p_callback
) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_area_monitor_callback(p_callback);
}

void JoltPhysicsServer3D::_area_set_ray_pickable(const RID& p_area, bool p_enable) {
	JoltAreaImpl3D* area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);

	area->set_pickable(p_enable);
}

RID JoltPhysicsServer3D::_body_create() {
	JoltBodyImpl3D* body = memnew(JoltBodyImpl3D);
	RID rid = body_owner.make_rid(body);
	body->set_rid(rid);
	return rid;
}

void JoltPhysicsServer3D::_body_set_space(const RID& p_body, const RID& p_space) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	JoltSpace3D* space = nullptr;

	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_NULL(space);
	}

	body->set_space(space);
}

RID JoltPhysicsServer3D::_body_get_space(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	const JoltSpace3D* space = body->get_space();

	if (space == nullptr) {
		return {};
	}

	return space->get_rid();
}

void JoltPhysicsServer3D::_body_set_mode(const RID& p_body, BodyMode p_mode) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_mode(p_mode);
}

PhysicsServer3D::BodyMode JoltPhysicsServer3D::_body_get_mode(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_mode();
}

void JoltPhysicsServer3D::_body_add_shape(
	const RID& p_body,
	const RID& p_shape,
	const Transform3D& p_transform,
	bool p_disabled
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	body->add_shape(shape, p_transform, p_disabled);
}

void JoltPhysicsServer3D::_body_set_shape(
	const RID& p_body,
	int32_t p_shape_idx,
	const RID& p_shape
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	JoltShapeImpl3D* shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);

	body->set_shape(p_shape_idx, shape);
}

void JoltPhysicsServer3D::_body_set_shape_transform(
	const RID& p_body,
	int32_t p_shape_idx,
	const Transform3D& p_transform
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_shape_transform(p_shape_idx, p_transform);
}

int32_t JoltPhysicsServer3D::_body_get_shape_count(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_shape_count();
}

RID JoltPhysicsServer3D::_body_get_shape(const RID& p_body, int32_t p_shape_idx) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	const JoltShapeImpl3D* shape = body->get_shape(p_shape_idx);
	ERR_FAIL_NULL_D(shape);

	return shape->get_rid();
}

Transform3D JoltPhysicsServer3D::_body_get_shape_transform(const RID& p_body, int32_t p_shape_idx)
	const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_shape_transform_scaled(p_shape_idx);
}

void JoltPhysicsServer3D::_body_remove_shape(const RID& p_body, int32_t p_shape_idx) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->remove_shape(p_shape_idx);
}

void JoltPhysicsServer3D::_body_clear_shapes(const RID& p_body) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->clear_shapes();
}

void JoltPhysicsServer3D::_body_set_shape_disabled(
	const RID& p_body,
	int32_t p_shape_idx,
	bool p_disabled
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_shape_disabled(p_shape_idx, p_disabled);
}

void JoltPhysicsServer3D::_body_attach_object_instance_id(const RID& p_body, uint64_t p_id) {
	if (JoltBodyImpl3D* body = body_owner.get_or_null(p_body)) {
		body->set_instance_id(ObjectID(p_id));
	} else if (JoltSoftBodyImpl3D* soft_body = soft_body_owner.get_or_null(p_body)) {
		soft_body->set_instance_id(ObjectID(p_id));
	} else {
		ERR_FAIL();
	}
}

uint64_t JoltPhysicsServer3D::_body_get_object_instance_id(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_instance_id();
}

void JoltPhysicsServer3D::_body_set_enable_continuous_collision_detection(
	const RID& p_body,
	bool p_enable
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_ccd_enabled(p_enable);
}

bool JoltPhysicsServer3D::_body_is_continuous_collision_detection_enabled(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->is_ccd_enabled();
}

void JoltPhysicsServer3D::_body_set_collision_layer(const RID& p_body, uint32_t p_layer) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_collision_layer(p_layer);
}

uint32_t JoltPhysicsServer3D::_body_get_collision_layer(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_collision_layer();
}

void JoltPhysicsServer3D::_body_set_collision_mask(const RID& p_body, uint32_t p_mask) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_collision_mask(p_mask);
}

uint32_t JoltPhysicsServer3D::_body_get_collision_mask(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_collision_mask();
}

void JoltPhysicsServer3D::_body_set_collision_priority(const RID& p_body, double p_priority) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_collision_priority((float)p_priority);
}

double JoltPhysicsServer3D::_body_get_collision_priority(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return (double)body->get_collision_priority();
}

void JoltPhysicsServer3D::_body_set_user_flags(
	[[maybe_unused]] const RID& p_body,
	[[maybe_unused]] uint32_t p_flags
) {
	WARN_PRINT(
		"Body user flags are not supported by Godot Jolt. "
		"Any such value will be ignored."
	);
}

uint32_t JoltPhysicsServer3D::_body_get_user_flags([[maybe_unused]] const RID& p_body) const {
	return 0;
}

void JoltPhysicsServer3D::_body_set_param(
	const RID& p_body,
	BodyParameter p_param,
	const Variant& p_value
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_param(p_param, p_value);
}

Variant JoltPhysicsServer3D::_body_get_param(const RID& p_body, BodyParameter p_param) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_param(p_param);
}

void JoltPhysicsServer3D::_body_reset_mass_properties(const RID& p_body) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->reset_mass_properties();
}

void JoltPhysicsServer3D::_body_set_state(
	const RID& p_body,
	BodyState p_state,
	const Variant& p_value
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_state(p_state, p_value);
}

Variant JoltPhysicsServer3D::_body_get_state(const RID& p_body, BodyState p_state) const {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_state(p_state);
}

void JoltPhysicsServer3D::_body_apply_central_impulse(const RID& p_body, const Vector3& p_impulse) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->apply_central_impulse(p_impulse);
}

void JoltPhysicsServer3D::_body_apply_impulse(
	const RID& p_body,
	const Vector3& p_impulse,
	const Vector3& p_position
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->apply_impulse(p_impulse, p_position);
}

void JoltPhysicsServer3D::_body_apply_torque_impulse(const RID& p_body, const Vector3& p_impulse) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->apply_torque_impulse(p_impulse);
}

void JoltPhysicsServer3D::_body_apply_central_force(const RID& p_body, const Vector3& p_force) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->apply_central_force(p_force);
}

void JoltPhysicsServer3D::_body_apply_force(
	const RID& p_body,
	const Vector3& p_force,
	const Vector3& p_position
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->apply_force(p_force, p_position);
}

void JoltPhysicsServer3D::_body_apply_torque(const RID& p_body, const Vector3& p_torque) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->apply_torque(p_torque);
}

void JoltPhysicsServer3D::_body_add_constant_central_force(
	const RID& p_body,
	const Vector3& p_force
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_constant_central_force(p_force);
}

void JoltPhysicsServer3D::_body_add_constant_force(
	const RID& p_body,
	const Vector3& p_force,
	const Vector3& p_position
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_constant_force(p_force, p_position);
}

void JoltPhysicsServer3D::_body_add_constant_torque(const RID& p_body, const Vector3& p_torque) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_constant_torque(p_torque);
}

void JoltPhysicsServer3D::_body_set_constant_force(const RID& p_body, const Vector3& p_force) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_constant_force(p_force);
}

Vector3 JoltPhysicsServer3D::_body_get_constant_force(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_constant_force();
}

void JoltPhysicsServer3D::_body_set_constant_torque(const RID& p_body, const Vector3& p_torque) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_constant_torque(p_torque);
}

Vector3 JoltPhysicsServer3D::_body_get_constant_torque(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_constant_torque();
}

void JoltPhysicsServer3D::_body_set_axis_velocity(
	const RID& p_body,
	const Vector3& p_axis_velocity
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_axis_velocity(p_axis_velocity);
}

void JoltPhysicsServer3D::_body_set_axis_lock(const RID& p_body, BodyAxis p_axis, bool p_lock) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_axis_lock(p_axis, p_lock);
}

bool JoltPhysicsServer3D::_body_is_axis_locked(const RID& p_body, BodyAxis p_axis) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->is_axis_locked(p_axis);
}

void JoltPhysicsServer3D::_body_add_collision_exception(
	const RID& p_body,
	const RID& p_excepted_body
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_collision_exception(p_excepted_body);
}

void JoltPhysicsServer3D::_body_remove_collision_exception(
	const RID& p_body,
	const RID& p_excepted_body
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->remove_collision_exception(p_excepted_body);
}

TypedArray<RID> JoltPhysicsServer3D::_body_get_collision_exceptions(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_collision_exceptions();
}

void JoltPhysicsServer3D::_body_set_max_contacts_reported(const RID& p_body, int32_t p_amount) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_max_contacts_reported(p_amount);
}

int32_t JoltPhysicsServer3D::_body_get_max_contacts_reported(const RID& p_body) const {
	const JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_max_contacts_reported();
}

void JoltPhysicsServer3D::_body_set_contacts_reported_depth_threshold(
	[[maybe_unused]] const RID& p_body,
	[[maybe_unused]] double p_threshold
) {
	WARN_PRINT(
		"Per-body contact depth threshold is not supported by Godot Jolt. "
		"Any such value will be ignored."
	);
}

double JoltPhysicsServer3D::_body_get_contacts_reported_depth_threshold(
	[[maybe_unused]] const RID& p_body
) const {
	return 0.0;
}

void JoltPhysicsServer3D::_body_set_omit_force_integration(const RID& p_body, bool p_enable) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_custom_integrator(p_enable);
}

bool JoltPhysicsServer3D::_body_is_omitting_force_integration(const RID& p_body) const {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->has_custom_integrator();
}

void JoltPhysicsServer3D::_body_set_state_sync_callback(
	const RID& p_body,
	const Callable& p_callable
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_state_sync_callback(p_callable);
}

void JoltPhysicsServer3D::_body_set_force_integration_callback(
	const RID& p_body,
	const Callable& p_callable,
	const Variant& p_userdata
) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_custom_integration_callback(p_callable, p_userdata);
}

void JoltPhysicsServer3D::_body_set_ray_pickable(const RID& p_body, bool p_enable) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_pickable(p_enable);
}

bool JoltPhysicsServer3D::_body_test_motion(
	const RID& p_body,
	const Transform3D& p_from,
	const Vector3& p_motion,
	double p_margin,
	int32_t p_max_collisions,
	bool p_collide_separation_ray,
	bool p_recovery_as_collision,
	PhysicsServer3DExtensionMotionResult* p_result
) const {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	JoltSpace3D* space = body->get_space();
	ERR_FAIL_NULL_D(space);

	return space->get_direct_state()->test_body_motion(
		*body,
		p_from,
		p_motion,
		(float)p_margin,
		p_max_collisions,
		p_collide_separation_ray,
		p_recovery_as_collision,
		p_result
	);
}

PhysicsDirectBodyState3D* JoltPhysicsServer3D::_body_get_direct_state(const RID& p_body) {
	JoltBodyImpl3D* body = body_owner.get_or_null(p_body);

	// Unlike most other server methods this one is meant to quietly return null if the body has
	// since been freed or removed from the scene tree, which is used in places like
	// `move_and_slide` to determine whether a previously used platform has been freed or not.
	QUIET_FAIL_NULL_D(body);
	QUIET_FAIL_NULL_D(body->get_space());

	return body->get_direct_state();
}

RID JoltPhysicsServer3D::_soft_body_create() {
	JoltSoftBodyImpl3D* body = memnew(JoltSoftBodyImpl3D);
	RID rid = soft_body_owner.make_rid(body);
	body->set_rid(rid);
	return rid;
}

void JoltPhysicsServer3D::_soft_body_update_rendering_server(
	const RID& p_body,
	PhysicsServer3DRenderingServerHandler* p_rendering_server_handler
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->update_rendering_server(p_rendering_server_handler);
}

void JoltPhysicsServer3D::_soft_body_set_space(const RID& p_body, const RID& p_space) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	JoltSpace3D* space = nullptr;

	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_NULL(space);
	}

	body->set_space(space);
}

RID JoltPhysicsServer3D::_soft_body_get_space(const RID& p_body) const {
	const JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	const JoltSpace3D* space = body->get_space();

	if (space == nullptr) {
		return {};
	}

	return space->get_rid();
}

void JoltPhysicsServer3D::_soft_body_set_mesh(const RID& p_body, const RID& p_mesh) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_mesh(p_mesh);
}

AABB JoltPhysicsServer3D::_soft_body_get_bounds(const RID& p_body) const {
	const JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_bounds();
}

void JoltPhysicsServer3D::_soft_body_set_collision_layer(const RID& p_body, uint32_t p_layer) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_collision_layer(p_layer);
}

uint32_t JoltPhysicsServer3D::_soft_body_get_collision_layer(const RID& p_body) const {
	const JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_collision_layer();
}

void JoltPhysicsServer3D::_soft_body_set_collision_mask(const RID& p_body, uint32_t p_mask) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_collision_mask(p_mask);
}

uint32_t JoltPhysicsServer3D::_soft_body_get_collision_mask(const RID& p_body) const {
	const JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_collision_mask();
}

void JoltPhysicsServer3D::_soft_body_add_collision_exception(
	const RID& p_body,
	const RID& p_excepted_body
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->add_collision_exception(p_excepted_body);
}

void JoltPhysicsServer3D::_soft_body_remove_collision_exception(
	const RID& p_body,
	const RID& p_excepted_body
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->remove_collision_exception(p_excepted_body);
}

TypedArray<RID> JoltPhysicsServer3D::_soft_body_get_collision_exceptions(const RID& p_body) const {
	const JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_collision_exceptions();
}

void JoltPhysicsServer3D::_soft_body_set_state(
	const RID& p_body,
	BodyState p_state,
	const Variant& p_value
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_state(p_state, p_value);
}

Variant JoltPhysicsServer3D::_soft_body_get_state(const RID& p_body, BodyState p_state) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_state(p_state);
}

void JoltPhysicsServer3D::_soft_body_set_transform(
	const RID& p_body,
	const Transform3D& p_transform
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_transform(p_transform);
}

void JoltPhysicsServer3D::_soft_body_set_ray_pickable(const RID& p_body, bool p_enable) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_pickable(p_enable);
}

void JoltPhysicsServer3D::_soft_body_set_simulation_precision(
	const RID& p_body,
	int32_t p_precision
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_simulation_precision(p_precision);
}

int32_t JoltPhysicsServer3D::_soft_body_get_simulation_precision(const RID& p_body) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_simulation_precision();
}

void JoltPhysicsServer3D::_soft_body_set_total_mass(const RID& p_body, double p_total_mass) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_mass((float)p_total_mass);
}

double JoltPhysicsServer3D::_soft_body_get_total_mass(const RID& p_body) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return (double)body->get_mass();
}

void JoltPhysicsServer3D::_soft_body_set_linear_stiffness(const RID& p_body, double p_coefficient) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_stiffness_coefficient((float)p_coefficient);
}

double JoltPhysicsServer3D::_soft_body_get_linear_stiffness(const RID& p_body) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return (double)body->get_stiffness_coefficient();
}

void JoltPhysicsServer3D::_soft_body_set_pressure_coefficient(
	const RID& p_body,
	double p_coefficient
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_pressure((float)p_coefficient);
}

double JoltPhysicsServer3D::_soft_body_get_pressure_coefficient(const RID& p_body) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return (double)body->get_pressure();
}

void JoltPhysicsServer3D::_soft_body_set_damping_coefficient(
	const RID& p_body,
	double p_coefficient
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_linear_damping((float)p_coefficient);
}

double JoltPhysicsServer3D::_soft_body_get_damping_coefficient(const RID& p_body) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return (double)body->get_linear_damping();
}

void JoltPhysicsServer3D::_soft_body_set_drag_coefficient(const RID& p_body, double p_coefficient) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	return body->set_drag((float)p_coefficient);
}

double JoltPhysicsServer3D::_soft_body_get_drag_coefficient(const RID& p_body) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return (double)body->get_drag();
}

void JoltPhysicsServer3D::_soft_body_move_point(
	const RID& p_body,
	int32_t p_point_index,
	const Vector3& p_global_position
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->set_vertex_position(p_point_index, p_global_position);
}

Vector3 JoltPhysicsServer3D::_soft_body_get_point_global_position(
	const RID& p_body,
	int32_t p_point_index
) const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->get_vertex_position(p_point_index);
}

void JoltPhysicsServer3D::_soft_body_remove_all_pinned_points(const RID& p_body) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	body->unpin_all_vertices();
}

void JoltPhysicsServer3D::_soft_body_pin_point(
	const RID& p_body,
	int32_t p_point_index,
	bool p_pin
) {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);

	if (p_pin) {
		body->pin_vertex(p_point_index);
	} else {
		body->unpin_vertex(p_point_index);
	}
}

bool JoltPhysicsServer3D::_soft_body_is_point_pinned(const RID& p_body, int32_t p_point_index)
	const {
	JoltSoftBodyImpl3D* body = soft_body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_D(body);

	return body->is_vertex_pinned(p_point_index);
}

RID JoltPhysicsServer3D::_joint_create() {
	JoltJointImpl3D* joint = memnew(JoltJointImpl3D);
	RID rid = joint_owner.make_rid(joint);
	joint->set_rid(rid);
	return rid;
}

void JoltPhysicsServer3D::_joint_clear(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	if (joint->get_type() != JOINT_TYPE_MAX) {
		JoltJointImpl3D* empty_joint = memnew(JoltJointImpl3D);
		empty_joint->set_rid(joint->get_rid());

		memdelete_safely(joint);
		joint_owner.replace(p_joint, empty_joint);
	}
}

void JoltPhysicsServer3D::_joint_make_pin(
	const RID& p_joint,
	const RID& p_body_a,
	const Vector3& p_local_a,
	const RID& p_body_b,
	const Vector3& p_local_b
) {
	JoltJointImpl3D* old_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(old_joint);

	JoltBodyImpl3D* body_a = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(body_a);

	JoltBodyImpl3D* body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(body_a == body_b);

	JoltJointImpl3D* new_joint = memnew(
		JoltPinJointImpl3D(*old_joint, body_a, body_b, p_local_a, p_local_b)
	);

	memdelete_safely(old_joint);
	joint_owner.replace(p_joint, new_joint);
}

void JoltPhysicsServer3D::_pin_joint_set_param(
	const RID& p_joint,
	PinJointParam p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_PIN);
	auto* pin_joint = static_cast<JoltPinJointImpl3D*>(joint);

	pin_joint->set_param(p_param, p_value);
}

double JoltPhysicsServer3D::_pin_joint_get_param(const RID& p_joint, PinJointParam p_param) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_PIN);
	const auto* pin_joint = static_cast<const JoltPinJointImpl3D*>(joint);

	return pin_joint->get_param(p_param);
}

void JoltPhysicsServer3D::_pin_joint_set_local_a(const RID& p_joint, const Vector3& p_local_a) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_PIN);
	auto* pin_joint = static_cast<JoltPinJointImpl3D*>(joint);

	pin_joint->set_local_a(p_local_a);
}

Vector3 JoltPhysicsServer3D::_pin_joint_get_local_a(const RID& p_joint) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_PIN);
	const auto* pin_joint = static_cast<const JoltPinJointImpl3D*>(joint);

	return pin_joint->get_local_a();
}

void JoltPhysicsServer3D::_pin_joint_set_local_b(const RID& p_joint, const Vector3& p_local_b) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_PIN);
	auto* pin_joint = static_cast<JoltPinJointImpl3D*>(joint);

	pin_joint->set_local_b(p_local_b);
}

Vector3 JoltPhysicsServer3D::_pin_joint_get_local_b(const RID& p_joint) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_PIN);
	const auto* pin_joint = static_cast<const JoltPinJointImpl3D*>(joint);

	return pin_joint->get_local_b();
}

void JoltPhysicsServer3D::_joint_make_hinge(
	const RID& p_joint,
	const RID& p_body_a,
	const Transform3D& p_hinge_a,
	const RID& p_body_b,
	const Transform3D& p_hinge_b
) {
	JoltJointImpl3D* old_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(old_joint);

	JoltBodyImpl3D* body_a = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(body_a);

	JoltBodyImpl3D* body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(body_a == body_b);

	JoltJointImpl3D* new_joint = memnew(
		JoltHingeJointImpl3D(*old_joint, body_a, body_b, p_hinge_a, p_hinge_b)
	);

	memdelete_safely(old_joint);
	joint_owner.replace(p_joint, new_joint);
}

void JoltPhysicsServer3D::_joint_make_hinge_simple(
	[[maybe_unused]] const RID& p_joint,
	[[maybe_unused]] const RID& p_body_a,
	[[maybe_unused]] const Vector3& p_pivot_a,
	[[maybe_unused]] const Vector3& p_axis_a,
	[[maybe_unused]] const RID& p_body_b,
	[[maybe_unused]] const Vector3& p_pivot_b,
	[[maybe_unused]] const Vector3& p_axis_b
) {
	// HACK(mihe): This method doesn't seem to be used anywhere within Godot, and isn't exposed in
	// the bindings, so this will be unsupported until anyone actually needs it.
	ERR_FAIL_MSG("Simple hinge joints are not supported by Godot Jolt.");
}

void JoltPhysicsServer3D::_hinge_joint_set_param(
	const RID& p_joint,
	HingeJointParam p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->set_param(p_param, p_value);
}

double JoltPhysicsServer3D::_hinge_joint_get_param(const RID& p_joint, HingeJointParam p_param)
	const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_HINGE);
	const auto* hinge_joint = static_cast<const JoltHingeJointImpl3D*>(joint);

	return hinge_joint->get_param(p_param);
}

void JoltPhysicsServer3D::_hinge_joint_set_flag(
	const RID& p_joint,
	HingeJointFlag p_flag,
	bool p_enabled
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->set_flag(p_flag, p_enabled);
}

bool JoltPhysicsServer3D::_hinge_joint_get_flag(const RID& p_joint, HingeJointFlag p_flag) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_HINGE);
	const auto* hinge_joint = static_cast<const JoltHingeJointImpl3D*>(joint);

	return hinge_joint->get_flag(p_flag);
}

void JoltPhysicsServer3D::_joint_make_slider(
	const RID& p_joint,
	const RID& p_body_a,
	const Transform3D& p_local_ref_a,
	const RID& p_body_b,
	const Transform3D& p_local_ref_b
) {
	JoltJointImpl3D* old_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(old_joint);

	JoltBodyImpl3D* body_a = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(body_a);

	JoltBodyImpl3D* body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(body_a == body_b);

	JoltJointImpl3D* new_joint = memnew(
		JoltSliderJointImpl3D(*old_joint, body_a, body_b, p_local_ref_a, p_local_ref_b)
	);

	memdelete_safely(old_joint);
	joint_owner.replace(p_joint, new_joint);
}

void JoltPhysicsServer3D::_slider_joint_set_param(
	const RID& p_joint,
	SliderJointParam p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_SLIDER);
	auto* slider_joint = static_cast<JoltSliderJointImpl3D*>(joint);

	return slider_joint->set_param(p_param, p_value);
}

double JoltPhysicsServer3D::_slider_joint_get_param(const RID& p_joint, SliderJointParam p_param)
	const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_SLIDER);
	const auto* slider_joint = static_cast<const JoltSliderJointImpl3D*>(joint);

	return slider_joint->get_param(p_param);
}

void JoltPhysicsServer3D::_joint_make_cone_twist(
	const RID& p_joint,
	const RID& p_body_a,
	const Transform3D& p_local_ref_a,
	const RID& p_body_b,
	const Transform3D& p_local_ref_b
) {
	JoltJointImpl3D* old_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(old_joint);

	JoltBodyImpl3D* body_a = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(body_a);

	JoltBodyImpl3D* body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(body_a == body_b);

	JoltJointImpl3D* new_joint = memnew(
		JoltConeTwistJointImpl3D(*old_joint, body_a, body_b, p_local_ref_a, p_local_ref_b)
	);

	memdelete_safely(old_joint);
	joint_owner.replace(p_joint, new_joint);
}

void JoltPhysicsServer3D::_cone_twist_joint_set_param(
	const RID& p_joint,
	ConeTwistJointParam p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	auto* cone_twist_joint = static_cast<JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->set_param(p_param, p_value);
}

double JoltPhysicsServer3D::_cone_twist_joint_get_param(
	const RID& p_joint,
	ConeTwistJointParam p_param
) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	const auto* cone_twist_joint = static_cast<const JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->get_param(p_param);
}

void JoltPhysicsServer3D::_joint_make_generic_6dof(
	const RID& p_joint,
	const RID& p_body_a,
	const Transform3D& p_local_ref_a,
	const RID& p_body_b,
	const Transform3D& p_local_ref_b
) {
	JoltJointImpl3D* old_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(old_joint);

	JoltBodyImpl3D* body_a = body_owner.get_or_null(p_body_a);
	ERR_FAIL_NULL(body_a);

	JoltBodyImpl3D* body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(body_a == body_b);

	JoltJointImpl3D* new_joint = memnew(
		JoltGeneric6DOFJointImpl3D(*old_joint, body_a, body_b, p_local_ref_a, p_local_ref_b)
	);

	memdelete_safely(old_joint);
	joint_owner.replace(p_joint, new_joint);
}

void JoltPhysicsServer3D::_generic_6dof_joint_set_param(
	const RID& p_joint,
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisParam p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->set_param(p_axis, p_param, p_value);
}

double JoltPhysicsServer3D::_generic_6dof_joint_get_param(
	const RID& p_joint,
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisParam p_param
) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_6DOF);
	const auto* g6dof_joint = static_cast<const JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->get_param(p_axis, p_param);
}

void JoltPhysicsServer3D::_generic_6dof_joint_set_flag(
	const RID& p_joint,
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisFlag p_flag,
	bool p_enable
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->set_flag(p_axis, p_flag, p_enable);
}

bool JoltPhysicsServer3D::_generic_6dof_joint_get_flag(
	const RID& p_joint,
	Vector3::Axis p_axis,
	PhysicsServer3D::G6DOFJointAxisFlag p_flag
) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_6DOF);
	const auto* g6dof_joint = static_cast<const JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->get_flag(p_axis, p_flag);
}

PhysicsServer3D::JointType JoltPhysicsServer3D::_joint_get_type(const RID& p_joint) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	return joint->get_type();
}

void JoltPhysicsServer3D::_joint_set_solver_priority(const RID& p_joint, int32_t p_priority) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	joint->set_solver_priority(p_priority);
}

int32_t JoltPhysicsServer3D::_joint_get_solver_priority(const RID& p_joint) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	return joint->get_solver_priority();
}

void JoltPhysicsServer3D::_joint_disable_collisions_between_bodies(
	const RID& p_joint,
	bool p_disable
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	joint->set_collision_disabled(p_disable);
}

bool JoltPhysicsServer3D::_joint_is_disabled_collisions_between_bodies(const RID& p_joint) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	return joint->is_collision_disabled();
}

void JoltPhysicsServer3D::_free_rid(const RID& p_rid) {
	if (JoltShapeImpl3D* shape = shape_owner.get_or_null(p_rid)) {
		free_shape(shape);
	} else if (JoltBodyImpl3D* body = body_owner.get_or_null(p_rid)) {
		free_body(body);
	} else if (JoltJointImpl3D* joint = joint_owner.get_or_null(p_rid)) {
		free_joint(joint);
	} else if (JoltAreaImpl3D* area = area_owner.get_or_null(p_rid)) {
		free_area(area);
	} else if (JoltSoftBodyImpl3D* soft_body = soft_body_owner.get_or_null(p_rid)) {
		free_soft_body(soft_body);
	} else if (JoltSpace3D* space = space_owner.get_or_null(p_rid)) {
		free_space(space);
	} else {
		ERR_FAIL_MSG("Failed to free RID: The specified RID has no owner.");
	}
}

void JoltPhysicsServer3D::_set_active(bool p_active) {
	active = p_active;
}

void JoltPhysicsServer3D::_init() {
	job_system = new JoltJobSystem();
}

void JoltPhysicsServer3D::_step(double p_step) {
	if (!active) {
		return;
	}

	for (JoltSpace3D* active_space : active_spaces) {
		job_system->pre_step();

		active_space->step((float)p_step);

		job_system->post_step();
	}
}

void JoltPhysicsServer3D::_flush_queries() {
	if (!active) {
		return;
	}

	flushing_queries = true;

	for (JoltSpace3D* space : active_spaces) {
		space->call_queries();
	}

	flushing_queries = false;

#ifdef GDJ_CONFIG_EDITOR
	job_system->flush_timings();
#endif // GDJ_CONFIG_EDITOR
}

void JoltPhysicsServer3D::_finish() {
	delete_safely(job_system);
}

bool JoltPhysicsServer3D::_is_flushing_queries() const {
	return flushing_queries;
}

int32_t JoltPhysicsServer3D::_get_process_info([[maybe_unused]] ProcessInfo p_process_info) {
	return 0;
}

void JoltPhysicsServer3D::free_space(JoltSpace3D* p_space) {
	ERR_FAIL_NULL(p_space);

	free_area(p_space->get_default_area());
	space_set_active(p_space->get_rid(), false);
	space_owner.free(p_space->get_rid());
	memdelete_safely(p_space);
}

void JoltPhysicsServer3D::free_area(JoltAreaImpl3D* p_area) {
	ERR_FAIL_NULL(p_area);

	p_area->set_space(nullptr);
	area_owner.free(p_area->get_rid());
	memdelete_safely(p_area);
}

void JoltPhysicsServer3D::free_body(JoltBodyImpl3D* p_body) {
	ERR_FAIL_NULL(p_body);

	p_body->set_space(nullptr);
	body_owner.free(p_body->get_rid());
	memdelete_safely(p_body);
}

void JoltPhysicsServer3D::free_soft_body(JoltSoftBodyImpl3D* p_body) {
	ERR_FAIL_NULL(p_body);

	p_body->set_space(nullptr);
	soft_body_owner.free(p_body->get_rid());
	memdelete_safely(p_body);
}

void JoltPhysicsServer3D::free_shape(JoltShapeImpl3D* p_shape) {
	ERR_FAIL_NULL(p_shape);

	p_shape->remove_self();
	shape_owner.free(p_shape->get_rid());
	memdelete_safely(p_shape);
}

void JoltPhysicsServer3D::free_joint(JoltJointImpl3D* p_joint) {
	ERR_FAIL_NULL(p_joint);

	joint_owner.free(p_joint->get_rid());
	memdelete_safely(p_joint);
}

#ifdef GDJ_CONFIG_EDITOR

void JoltPhysicsServer3D::dump_debug_snapshots(const String& p_dir) {
	for (JoltSpace3D* space : active_spaces) {
		space->dump_debug_snapshot(p_dir);
	}
}

void JoltPhysicsServer3D::space_dump_debug_snapshot(const RID& p_space, const String& p_dir) {
	JoltSpace3D* space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);

	space->dump_debug_snapshot(p_dir);
}

#endif // GDJ_CONFIG_EDITOR

bool JoltPhysicsServer3D::joint_get_enabled(const RID& p_joint) const {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	return joint->is_enabled();
}

void JoltPhysicsServer3D::joint_set_enabled(const RID& p_joint, bool p_enabled) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	joint->set_enabled(p_enabled);
}

int32_t JoltPhysicsServer3D::joint_get_solver_velocity_iterations(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	return joint->get_solver_velocity_iterations();
}

void JoltPhysicsServer3D::joint_set_solver_velocity_iterations(
	const RID& p_joint,
	int32_t p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	return joint->set_solver_velocity_iterations(p_value);
}

int32_t JoltPhysicsServer3D::joint_get_solver_position_iterations(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	return joint->get_solver_position_iterations();
}

void JoltPhysicsServer3D::joint_set_solver_position_iterations(
	const RID& p_joint,
	int32_t p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	return joint->set_solver_position_iterations(p_value);
}

float JoltPhysicsServer3D::pin_joint_get_applied_force(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_PIN);
	auto* pin_joint = static_cast<JoltPinJointImpl3D*>(joint);

	return pin_joint->get_applied_force();
}

double JoltPhysicsServer3D::hinge_joint_get_jolt_param(
	const RID& p_joint,
	HingeJointParamJolt p_param
) const {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->get_jolt_param(p_param);
}

void JoltPhysicsServer3D::hinge_joint_set_jolt_param(
	const RID& p_joint,
	HingeJointParamJolt p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->set_jolt_param(p_param, p_value);
}

bool JoltPhysicsServer3D::hinge_joint_get_jolt_flag(const RID& p_joint, HingeJointFlagJolt p_flag)
	const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_HINGE);
	const auto* hinge_joint = static_cast<const JoltHingeJointImpl3D*>(joint);

	return hinge_joint->get_jolt_flag(p_flag);
}

void JoltPhysicsServer3D::hinge_joint_set_jolt_flag(
	const RID& p_joint,
	HingeJointFlagJolt p_flag,
	bool p_enabled
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->set_jolt_flag(p_flag, p_enabled);
}

float JoltPhysicsServer3D::hinge_joint_get_applied_force(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->get_applied_force();
}

float JoltPhysicsServer3D::hinge_joint_get_applied_torque(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_HINGE);
	auto* hinge_joint = static_cast<JoltHingeJointImpl3D*>(joint);

	return hinge_joint->get_applied_torque();
}

double JoltPhysicsServer3D::slider_joint_get_jolt_param(
	const RID& p_joint,
	SliderJointParamJolt p_param
) const {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_SLIDER);
	auto* slider_joint = static_cast<JoltSliderJointImpl3D*>(joint);

	return slider_joint->get_jolt_param(p_param);
}

void JoltPhysicsServer3D::slider_joint_set_jolt_param(
	const RID& p_joint,
	SliderJointParamJolt p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_SLIDER);
	auto* slider_joint = static_cast<JoltSliderJointImpl3D*>(joint);

	return slider_joint->set_jolt_param(p_param, p_value);
}

bool JoltPhysicsServer3D::slider_joint_get_jolt_flag(const RID& p_joint, SliderJointFlagJolt p_flag)
	const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_SLIDER);
	const auto* slider_joint = static_cast<const JoltSliderJointImpl3D*>(joint);

	return slider_joint->get_jolt_flag(p_flag);
}

void JoltPhysicsServer3D::slider_joint_set_jolt_flag(
	const RID& p_joint,
	SliderJointFlagJolt p_flag,
	bool p_enabled
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_SLIDER);
	auto* slider_joint = static_cast<JoltSliderJointImpl3D*>(joint);

	return slider_joint->set_jolt_flag(p_flag, p_enabled);
}

float JoltPhysicsServer3D::slider_joint_get_applied_force(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_SLIDER);
	auto* slider_joint = static_cast<JoltSliderJointImpl3D*>(joint);

	return slider_joint->get_applied_force();
}

float JoltPhysicsServer3D::slider_joint_get_applied_torque(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_SLIDER);
	auto* slider_joint = static_cast<JoltSliderJointImpl3D*>(joint);

	return slider_joint->get_applied_torque();
}

double JoltPhysicsServer3D::cone_twist_joint_get_jolt_param(
	const RID& p_joint,
	ConeTwistJointParamJolt p_param
) const {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	auto* cone_twist_joint = static_cast<JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->get_jolt_param(p_param);
}

void JoltPhysicsServer3D::cone_twist_joint_set_jolt_param(
	const RID& p_joint,
	ConeTwistJointParamJolt p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	auto* cone_twist_joint = static_cast<JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->set_jolt_param(p_param, p_value);
}

bool JoltPhysicsServer3D::cone_twist_joint_get_jolt_flag(
	const RID& p_joint,
	ConeTwistJointFlagJolt p_flag
) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	const auto* cone_twist_joint = static_cast<const JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->get_jolt_flag(p_flag);
}

void JoltPhysicsServer3D::cone_twist_joint_set_jolt_flag(
	const RID& p_joint,
	ConeTwistJointFlagJolt p_flag,
	bool p_enabled
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	auto* cone_twist_joint = static_cast<JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->set_jolt_flag(p_flag, p_enabled);
}

float JoltPhysicsServer3D::cone_twist_joint_get_applied_force(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	auto* cone_twist_joint = static_cast<JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->get_applied_force();
}

float JoltPhysicsServer3D::cone_twist_joint_get_applied_torque(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_CONE_TWIST);
	auto* cone_twist_joint = static_cast<JoltConeTwistJointImpl3D*>(joint);

	return cone_twist_joint->get_applied_torque();
}

double JoltPhysicsServer3D::generic_6dof_joint_get_jolt_param(
	const RID& p_joint,
	Vector3::Axis p_axis,
	G6DOFJointAxisParamJolt p_param
) const {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->get_jolt_param(p_axis, p_param);
}

void JoltPhysicsServer3D::generic_6dof_joint_set_jolt_param(
	const RID& p_joint,
	Vector3::Axis p_axis,
	G6DOFJointAxisParamJolt p_param,
	double p_value
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->set_jolt_param(p_axis, p_param, p_value);
}

bool JoltPhysicsServer3D::generic_6dof_joint_get_jolt_flag(
	const RID& p_joint,
	Vector3::Axis p_axis,
	G6DOFJointAxisFlagJolt p_flag
) const {
	const JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_6DOF);
	const auto* g6dof_joint = static_cast<const JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->get_jolt_flag(p_axis, p_flag);
}

void JoltPhysicsServer3D::generic_6dof_joint_set_jolt_flag(
	const RID& p_joint,
	Vector3::Axis p_axis,
	G6DOFJointAxisFlagJolt p_flag,
	bool p_enabled
) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);

	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->set_jolt_flag(p_axis, p_flag, p_enabled);
}

float JoltPhysicsServer3D::generic_6dof_joint_get_applied_force(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->get_applied_force();
}

float JoltPhysicsServer3D::generic_6dof_joint_get_applied_torque(const RID& p_joint) {
	JoltJointImpl3D* joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_D(joint);

	ERR_FAIL_COND_D(joint->get_type() != JOINT_TYPE_6DOF);
	auto* g6dof_joint = static_cast<JoltGeneric6DOFJointImpl3D*>(joint);

	return g6dof_joint->get_applied_torque();
}
