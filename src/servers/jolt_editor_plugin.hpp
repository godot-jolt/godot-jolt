#pragma once

#ifdef GDJ_CONFIG_EDITOR

#include "joints/jolt_joint_gizmo_plugin_3d.hpp"

class JoltEditorPlugin final : public EditorPlugin {
	GDCLASS_NO_WARN(JoltEditorPlugin, EditorPlugin)

private:
	static void _bind_methods() { }

public:
	void _enter_tree() override;

	void _exit_tree() override;

private:
	enum MenuOption {
		MENU_OPTION_DUMP_DEBUG_SNAPSHOTS
	};

	void _tool_menu_pressed(int32_t p_index);

	void _snapshots_dir_selected(const String& p_dir);

	void _dump_debug_snapshots();

	Ref<JoltJointGizmoPlugin3D> joint_gizmo_plugin;

	EditorFileDialog* debug_snapshots_dialog = nullptr;
};

#endif // GDJ_CONFIG_EDITOR
