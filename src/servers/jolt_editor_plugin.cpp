#include "jolt_editor_plugin.hpp"

#ifdef GDJ_CONFIG_EDITOR

#include "servers/jolt_physics_server_3d.hpp"

void JoltEditorPlugin::_bind_methods() {
	BIND_METHOD(JoltEditorPlugin, _tool_menu_pressed);
	BIND_METHOD(JoltEditorPlugin, _snapshots_dir_selected);
}

void JoltEditorPlugin::_enter_tree() {
	EditorInterface* editor_interface = get_editor_interface();
	Control* base_control = editor_interface->get_base_control();

	Ref<Theme> old_theme = base_control->get_theme();

	if (old_theme.is_valid()) {
		// HACK(mihe): For whatever reason the editor startup time takes a significant hit with
		// every icon added unless we duplicate the theme first and manipulate that instead.
		Ref<Theme> new_theme = old_theme->duplicate();

		Ref<Texture2D> icon_pin = new_theme->get_icon("PinJoint3D", "EditorIcons");
		Ref<Texture2D> icon_hinge = new_theme->get_icon("HingeJoint3D", "EditorIcons");
		Ref<Texture2D> icon_slider = new_theme->get_icon("SliderJoint3D", "EditorIcons");
		Ref<Texture2D> icon_cone_twist = new_theme->get_icon("ConeTwistJoint3D", "EditorIcons");
		Ref<Texture2D> icon_6dof = new_theme->get_icon("Generic6DOFJoint3D", "EditorIcons");

		new_theme->set_icon("JoltPinJoint3D", "EditorIcons", icon_pin);
		new_theme->set_icon("JoltHingeJoint3D", "EditorIcons", icon_hinge);
		new_theme->set_icon("JoltSliderJoint3D", "EditorIcons", icon_slider);
		new_theme->set_icon("JoltConeTwistJoint3D", "EditorIcons", icon_cone_twist);
		new_theme->set_icon("JoltGeneric6DOFJoint3D", "EditorIcons", icon_6dof);

		base_control->set_theme(new_theme);
	}

	joint_gizmo_plugin = Ref(memnew(JoltJointGizmoPlugin3D(editor_interface)));
	add_node_3d_gizmo_plugin(joint_gizmo_plugin);

	PopupMenu* tool_menu = memnew(PopupMenu);
	tool_menu->connect("id_pressed", Callable(this, NAMEOF(_tool_menu_pressed)));
	tool_menu->add_item("Dump Debug Snapshots", MENU_OPTION_DUMP_DEBUG_SNAPSHOTS);

	add_tool_submenu_item("Jolt Physics", tool_menu);
}

void JoltEditorPlugin::_exit_tree() {
	remove_node_3d_gizmo_plugin(joint_gizmo_plugin);
	joint_gizmo_plugin.unref();

	if (debug_snapshots_dialog != nullptr) {
		debug_snapshots_dialog->queue_free();
		debug_snapshots_dialog = nullptr;
	}
}

void JoltEditorPlugin::_tool_menu_pressed(int32_t p_index) {
	// NOLINTNEXTLINE(hicpp-multiway-paths-covered)
	switch (p_index) {
		case MENU_OPTION_DUMP_DEBUG_SNAPSHOTS: {
			_dump_debug_snapshots();
		} break;
	}
}

void JoltEditorPlugin::_snapshots_dir_selected(const String& p_dir) {
	auto* physics_server = static_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());
	physics_server->dump_debug_snapshots(p_dir);
}

void JoltEditorPlugin::_dump_debug_snapshots() {
	if (debug_snapshots_dialog == nullptr) {
		debug_snapshots_dialog = memnew(EditorFileDialog);
		debug_snapshots_dialog->set_file_mode(EditorFileDialog::FILE_MODE_OPEN_DIR);
		debug_snapshots_dialog->set_access(EditorFileDialog::ACCESS_FILESYSTEM);
		debug_snapshots_dialog->set_current_dir("res://");
		debug_snapshots_dialog->connect(
			"dir_selected",
			Callable(this, NAMEOF(_snapshots_dir_selected))
		);

		get_editor_interface()->get_base_control()->add_child(debug_snapshots_dialog);
	}

	debug_snapshots_dialog->popup_centered_ratio(0.5);
}

#endif // GDJ_CONFIG_EDITOR
