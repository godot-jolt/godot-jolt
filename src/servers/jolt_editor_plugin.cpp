#include "jolt_editor_plugin.hpp"

#ifdef GDJ_CONFIG_EDITOR

void JoltEditorPlugin::_enter_tree() {
	EditorInterface* editor_interface = get_editor_interface();
	Control* base_control = editor_interface->get_base_control();

	// HACK(mihe): For whatever reason the editor startup time takes a significant hit with every
	// icon added unless we duplicate the theme first and manipulate that instead.
	Ref<Theme> theme = base_control->get_theme()->duplicate();

	Ref<Texture2D> icon_pin = theme->get_icon("PinJoint3D", "EditorIcons");
	Ref<Texture2D> icon_hinge = theme->get_icon("HingeJoint3D", "EditorIcons");
	Ref<Texture2D> icon_slider = theme->get_icon("SliderJoint3D", "EditorIcons");
	Ref<Texture2D> icon_cone_twist = theme->get_icon("ConeTwistJoint3D", "EditorIcons");
	Ref<Texture2D> icon_6dof = theme->get_icon("Generic6DOFJoint3D", "EditorIcons");

	theme->set_icon("JoltPinJoint3D", "EditorIcons", icon_pin);
	theme->set_icon("JoltHingeJoint3D", "EditorIcons", icon_hinge);
	theme->set_icon("JoltSliderJoint3D", "EditorIcons", icon_slider);
	theme->set_icon("JoltConeTwistJoint3D", "EditorIcons", icon_cone_twist);
	theme->set_icon("JoltGeneric6DOFJoint3D", "EditorIcons", icon_6dof);

	base_control->set_theme(theme);

	joint_gizmo_plugin = Ref(memnew(JoltJointGizmoPlugin3D(editor_interface)));
	add_node_3d_gizmo_plugin(joint_gizmo_plugin);
}

void JoltEditorPlugin::_exit_tree() {
	remove_node_3d_gizmo_plugin(joint_gizmo_plugin);
	joint_gizmo_plugin.unref();
}

#endif // GDJ_CONFIG_EDITOR
