#include "jolt_joint_gizmo_plugin_3d.hpp"

#ifdef GDJ_CONFIG_EDITOR

#include "joints/jolt_pin_joint_3d.hpp"

bool JoltJointGizmoPlugin3D::_has_gizmo(Node3D* p_node) const {
	return Object::cast_to<JoltJoint3D>(p_node) != nullptr;
}

String JoltJointGizmoPlugin3D::_get_gizmo_name() const {
	return U"JoltJoint3D";
}

void JoltJointGizmoPlugin3D::_redraw(const Ref<EditorNode3DGizmo>& p_gizmo) {
	auto* joint = Object::cast_to<JoltJoint3D>(p_gizmo->get_node_3d());

	p_gizmo->clear();

	PhysicsBody3D* body_a = joint->get_body_a();
	QUIET_FAIL_NULL(body_a);

	PhysicsBody3D* body_b = joint->get_body_b();
	QUIET_FAIL_NULL(body_b);

	if (!created_materials) {
		// HACK(mihe): Ideally we would do this in the constructor, but the documentation generation
		// will instantiate this too early in the program's flow, leading to a bunch of errors about
		// missing editor settings.

		create_material(U"joint", Color(0.5f, 0.8f, 1.0f));
		create_material(U"joint_body_a", Color(0.6f, 0.8f, 1.0f));
		create_material(U"joint_body_b", Color(0.6f, 0.9f, 1.0f));

		created_materials = true;
	}

	Ref<Material> material_common = get_material(U"joint", p_gizmo);
	Ref<Material> material_body_a = get_material(U"joint_body_a", p_gizmo);
	Ref<Material> material_body_b = get_material(U"joint_body_b", p_gizmo);

	if (auto* pin_joint = Object::cast_to<JoltPinJoint3D>(joint)) {
		constexpr float size = 0.25f;

		PackedVector3Array points;
		points.resize(6);

		points[0] = Vector3(+size, 0, 0);
		points[1] = Vector3(-size, 0, 0);
		points[2] = Vector3(0, +size, 0);
		points[3] = Vector3(0, -size, 0);
		points[4] = Vector3(0, 0, +size);
		points[5] = Vector3(0, 0, -size);

		p_gizmo->add_collision_segments(points);
		p_gizmo->add_lines(points, material_common);
	}
}

#endif // GDJ_CONFIG_EDITOR
