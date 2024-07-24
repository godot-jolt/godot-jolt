#include "math.hpp"

namespace godot::Math {

void decompose(Basis& p_basis, Vector3& p_scale) {
	const real_t sign = SIGN(p_basis.determinant());

	Vector3 x = p_basis.get_column(Vector3::AXIS_X);
	Vector3 y = p_basis.get_column(Vector3::AXIS_Y);
	Vector3 z = p_basis.get_column(Vector3::AXIS_Z);

	const real_t x_dot_x = x.dot(x);

	y -= x * (y.dot(x) / x_dot_x);
	z -= x * (z.dot(x) / x_dot_x);

	const real_t y_dot_y = y.dot(y);

	z -= y * (z.dot(y) / y_dot_y);

	const real_t z_dot_z = z.dot(z);

	p_scale = sign * Vector3(Math::sqrt(x_dot_x), Math::sqrt(y_dot_y), Math::sqrt(z_dot_z));

	p_basis.set_column(Vector3::AXIS_X, x / p_scale.x);
	p_basis.set_column(Vector3::AXIS_Y, y / p_scale.y);
	p_basis.set_column(Vector3::AXIS_Z, z / p_scale.z);
}

} // namespace godot::Math
