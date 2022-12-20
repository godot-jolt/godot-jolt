#include "jolt_physics_direct_body_state_3d.hpp"
#include "jolt_physics_server_3d.hpp"
#include "jolt_physics_server_factory_3d.hpp"

namespace {

constexpr ModuleInitializationLevel GDJOLT_INIT_LEVEL = MODULE_INITIALIZATION_LEVEL_SERVERS;

JoltPhysicsServerFactory3D* server_factory = nullptr;

void on_initialize(ModuleInitializationLevel p_level) {
	if (p_level != GDJOLT_INIT_LEVEL) {
		return;
	}

	ClassDB::register_class<JoltPhysicsDirectBodyState3D>();
	ClassDB::register_class<JoltPhysicsServer3D>();
	ClassDB::register_class<JoltPhysicsServerFactory3D>();

	if (server_factory == nullptr) {
		server_factory = memnew(JoltPhysicsServerFactory3D);
	}

	PhysicsServer3DManager::get_singleton()->register_server(
		"Jolt",
		Callable(server_factory, "create_server")
	);
}

void on_terminate(ModuleInitializationLevel p_level) {
	if (p_level != GDJOLT_INIT_LEVEL) {
		return;
	}

	if (server_factory != nullptr) {
		memdelete(server_factory);
		server_factory = nullptr;
	}
}

} // namespace

extern "C" {

GDExtensionBool GDE_EXPORT godot_jolt_main(
	GDExtensionInterface* p_interface,
	GDExtensionClassLibraryPtr p_class_library,
	GDExtensionInitialization* p_initialization
) {
	const GDExtensionBinding::InitObject init_obj(p_interface, p_class_library, p_initialization);

	init_obj.register_initializer(&on_initialize);
	init_obj.register_terminator(&on_terminate);

	init_obj.set_minimum_library_initialization_level(GDJOLT_INIT_LEVEL);

	return init_obj.init();
}

} // extern "C"
