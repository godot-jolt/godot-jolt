#include "jolt_world_boundary_shape_impl_3d.hpp"

Variant JoltWorldBoundaryShapeImpl3D::get_data() const {
	return plane;
}

void JoltWorldBoundaryShapeImpl3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		_invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::PLANE);

	plane = p_data;
}

JPH::ShapeRefC JoltWorldBoundaryShapeImpl3D::_build() const {
	ERR_FAIL_D_MSG(vformat(
		"WorldBoundaryShape3D is not supported by Godot Jolt. "
		"Consider using one or more reasonably sized BoxShape3D instead. "
		"This shape belongs to %s.",
		_owners_to_string()
	));
}
