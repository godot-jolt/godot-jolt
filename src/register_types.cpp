#include "joints/jolt_cone_twist_joint_3d.hpp"
#include "joints/jolt_generic_6dof_joint.hpp"
#include "joints/jolt_hinge_joint_3d.hpp"
#include "joints/jolt_joint_gizmo_plugin_3d.hpp"
#include "joints/jolt_pin_joint_3d.hpp"
#include "joints/jolt_slider_joint_3d.hpp"
#include "objects/jolt_physics_direct_body_state_3d.hpp"
#include "servers/jolt_editor_plugin.hpp"
#include "servers/jolt_globals.hpp"
#include "servers/jolt_physics_server_3d.hpp"
#include "servers/jolt_physics_server_factory_3d.hpp"
#include "servers/jolt_project_settings.hpp"
#include "spaces/jolt_debug_geometry_3d.hpp"
#include "spaces/jolt_physics_direct_space_state_3d.hpp"

namespace {

JoltPhysicsServerFactory3D* server_factory = nullptr;

void on_initialize(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_CORE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			jolt_initialize();

			ClassDB::register_class<JoltPhysicsDirectBodyState3D>();
			ClassDB::register_class<JoltPhysicsDirectSpaceState3D>();
			ClassDB::register_class<JoltPhysicsServer3D>();
			ClassDB::register_class<JoltPhysicsServerFactory3D>();

			server_factory = memnew(JoltPhysicsServerFactory3D);

			PhysicsServer3DManager::get_singleton()->register_server(
				"JoltPhysics3D",
				Callable(server_factory, "create_server")
			);
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			JoltProjectSettings::register_settings();

			ClassDB::register_class<JoltDebugGeometry3D>();
			ClassDB::register_class<JoltJoint3D>(true);
			ClassDB::register_class<JoltPinJoint3D>();
			ClassDB::register_class<JoltHingeJoint3D>();
			ClassDB::register_class<JoltSliderJoint3D>();
			ClassDB::register_class<JoltConeTwistJoint3D>();
			ClassDB::register_class<JoltGeneric6DOFJoint3D>();
		} break;
		case MODULE_INITIALIZATION_LEVEL_EDITOR: {
#ifdef GDJ_CONFIG_EDITOR
			ClassDB::register_class<JoltJointGizmoPlugin3D>();
			ClassDB::register_class<JoltEditorPlugin>();
			EditorPlugins::add_by_type<JoltEditorPlugin>();
#endif // GDJ_CONFIG_EDITOR
		} break;
	}
}

void on_terminate(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_CORE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			memdelete_safely(server_factory);

			jolt_deinitialize();
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_EDITOR: {
		} break;
	}
}

} // namespace

extern "C" {

GDExtensionBool GDE_EXPORT godot_jolt_main(
	GDExtensionInterfaceGetProcAddress p_get_proc_address,
	GDExtensionClassLibraryPtr p_library,
	GDExtensionInitialization* p_initialization
) {
	const GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, p_initialization);

	init_obj.register_initializer(&on_initialize);
	init_obj.register_terminator(&on_terminate);

	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}

} // extern "C"
