#pragma once

#ifdef GDJ_CONFIG_EDITOR

class JoltJointGizmoPlugin3D final : public EditorNode3DGizmoPlugin {
	GDCLASS_NO_WARN(JoltJointGizmoPlugin3D, EditorNode3DGizmoPlugin)

private:
	static void _bind_methods() { }

public:
	bool _has_gizmo(Node3D* p_node) const override;

	String _get_gizmo_name() const override;

	void _redraw(const Ref<EditorNode3DGizmo>& p_gizmo) override;

private:
	bool created_materials = false;
};

#endif // GDJ_CONFIG_EDITOR
