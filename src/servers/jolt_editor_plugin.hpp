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
	Ref<JoltJointGizmoPlugin3D> joint_gizmo_plugin;
};

#endif // GDJ_CONFIG_EDITOR
