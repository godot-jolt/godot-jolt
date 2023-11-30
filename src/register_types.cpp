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

#define ERR_PRINT_EARLY(m_msg) \
	internal::gdextension_interface_print_error(m_msg, __FUNCTION__, __FILE__, __LINE__, false)

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
		case MODULE_INITIALIZATION_LEVEL_MAX: {
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

	GDExtensionBool success = init_obj.init();

	if (success == 0) {
		return success;
	}

	if (internal::godot_version.major != GDJ_GODOT_VERSION_MAJOR ||
		internal::godot_version.minor != GDJ_GODOT_VERSION_MINOR)
	{
		char error_message[4096] = {'\0'};

		snprintf(
			error_message,
			(size_t)count_of(error_message),
			"Godot Jolt failed to load due to not supporting Godot %d.%d. "
			"This version of Godot Jolt (%d.%d.%d) only supports Godot %d.%d.",
			internal::godot_version.major,
			internal::godot_version.minor,
			GDJ_VERSION_MAJOR,
			GDJ_VERSION_MINOR,
			GDJ_VERSION_PATCH,
			GDJ_GODOT_VERSION_MAJOR,
			GDJ_GODOT_VERSION_MINOR
		);

		ERR_PRINT_EARLY(error_message);

		success = 0;
	}

	return success;
}

} // extern "C"
