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
#include "servers/jolt_project_settings.hpp"
#include "spaces/jolt_debug_geometry_3d.hpp"
#include "spaces/jolt_physics_direct_space_state_3d.hpp"

namespace {

JoltPhysicsServer3DExtension* create_jolt_physics_server() {
	return memnew(JoltPhysicsServer3DExtension);
}

void on_initialize(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_CORE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			jolt_initialize();

			GDREGISTER_VIRTUAL_CLASS(JoltPhysicsDirectBodyState3DExtension);
			GDREGISTER_VIRTUAL_CLASS(JoltPhysicsDirectSpaceState3DExtension);
			GDREGISTER_VIRTUAL_CLASS(JoltPhysicsServer3DExtension);

			PhysicsServer3DManager::get_singleton()->register_server(
				"Jolt Physics (Extension)",
				callable_mp_static(&create_jolt_physics_server)
			);
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			JoltProjectSettings::register_settings();

			GDREGISTER_VIRTUAL_CLASS(JoltJoint3D);
			GDREGISTER_CLASS(JoltPinJoint3D);
			GDREGISTER_CLASS(JoltHingeJoint3D);
			GDREGISTER_CLASS(JoltSliderJoint3D);
			GDREGISTER_CLASS(JoltConeTwistJoint3D);
			GDREGISTER_CLASS(JoltGeneric6DOFJoint3D);

#ifdef GDJ_CONFIG_DISTRIBUTION
			GDREGISTER_INTERNAL_CLASS(JoltDebugGeometry3D);
#else // GDJ_CONFIG_DISTRIBUTION
			GDREGISTER_CLASS(JoltDebugGeometry3D);
#endif // GDJ_CONFIG_DISTRIBUTION
		} break;
		case MODULE_INITIALIZATION_LEVEL_EDITOR: {
#ifdef GDJ_CONFIG_EDITOR
			GDREGISTER_INTERNAL_CLASS(JoltJointGizmoPlugin3D);
			GDREGISTER_INTERNAL_CLASS(JoltEditorPlugin);
			EditorPlugins::add_by_type<JoltEditorPlugin>();
#endif // GDJ_CONFIG_EDITOR
		} break;
		case MODULE_INITIALIZATION_LEVEL_MAX: {
		} break;
	}
}

void on_terminate(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_CORE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			jolt_deinitialize();
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_EDITOR: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_MAX: {
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
