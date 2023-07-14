#include "jolt_shape_impl_3d.hpp"

#include "objects/jolt_object_impl_3d.hpp"
#include "shapes/jolt_custom_user_data_shape.hpp"

namespace {

constexpr float DEFAULT_SOLVER_BIAS = 0.0;

} // namespace

JoltShapeImpl3D::~JoltShapeImpl3D() = default;

void JoltShapeImpl3D::add_owner(JoltObjectImpl3D* p_owner) {
	ref_counts_by_owner[p_owner]++;
}

void JoltShapeImpl3D::remove_owner(JoltObjectImpl3D* p_owner) {
	if (--ref_counts_by_owner[p_owner] <= 0) {
		ref_counts_by_owner.erase(p_owner);
	}
}

void JoltShapeImpl3D::remove_self(bool p_lock) {
	// `remove_owner` will be called when we `remove_shape`, so we need to copy the map since the
	// iterator would be invalidated from underneath us
	const auto ref_counts_by_owner_copy = ref_counts_by_owner;

	for (const auto& [owner, ref_count] : ref_counts_by_owner_copy) {
		owner->remove_shape(this, p_lock);
	}
}

float JoltShapeImpl3D::get_solver_bias() const {
	return DEFAULT_SOLVER_BIAS;
}

void JoltShapeImpl3D::set_solver_bias(float p_bias) {
	if (!Math::is_equal_approx(p_bias, DEFAULT_SOLVER_BIAS)) {
		WARN_PRINT(vformat(
			"Custom solver bias for shapes is not supported by Godot Jolt. "
			"Any such value will be ignored. "
			"This shape belongs to %s.",
			_owners_to_string()
		));
	}
}

JPH::ShapeRefC JoltShapeImpl3D::try_build() {
	if (jolt_ref == nullptr) {
		jolt_ref = _build();
	}

	return jolt_ref;
}

JPH::ShapeRefC JoltShapeImpl3D::with_scale(const JPH::Shape* p_shape, const Vector3& p_scale) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::ScaledShapeSettings shape_settings(p_shape, to_jolt(p_scale));
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to scale shape with {scale=%v}. "
			"It returned the following error: '%s'.",
			p_scale,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShapeImpl3D::with_basis_origin(
	const JPH::Shape* p_shape,
	const Basis& p_basis,
	const Vector3& p_origin
) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::RotatedTranslatedShapeSettings shape_settings(
		to_jolt(p_origin),
		to_jolt(p_basis),
		p_shape
	);

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to offset shape with {basis=%s origin=%v}. "
			"It returned the following error: '%s'.",
			p_basis,
			p_origin,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShapeImpl3D::with_transform(
	const JPH::Shape* p_shape,
	const Transform3D& p_transform,
	const Vector3& p_scale
) {
	ERR_FAIL_NULL_D(p_shape);

	JPH::ShapeRefC shape = p_shape;

	if (p_scale != Vector3(1.0f, 1.0f, 1.0f)) {
		shape = with_scale(shape, p_scale);
	}

	if (p_transform != Transform3D()) {
		shape = with_basis_origin(shape, p_transform.basis, p_transform.origin);
	}

	return shape;
}

JPH::ShapeRefC JoltShapeImpl3D::with_center_of_mass_offset(
	const JPH::Shape* p_shape,
	const Vector3& p_offset
) {
	ERR_FAIL_NULL_D(p_shape);

	const JPH::OffsetCenterOfMassShapeSettings shape_settings(to_jolt(p_offset), p_shape);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to offset center of mass with {offset=%v}. "
			"It returned the following error: '%s'.",
			p_offset,
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

JPH::ShapeRefC JoltShapeImpl3D::with_center_of_mass(
	const JPH::Shape* p_shape,
	const Vector3& p_center_of_mass
) {
	ERR_FAIL_NULL_D(p_shape);

	const Vector3 center_of_mass_inner = to_godot(p_shape->GetCenterOfMass());
	const Vector3 center_of_mass_offset = p_center_of_mass - center_of_mass_inner;

	if (center_of_mass_offset == Vector3()) {
		return p_shape;
	}

	return with_center_of_mass_offset(p_shape, center_of_mass_offset);
}

JPH::ShapeRefC JoltShapeImpl3D::with_user_data(const JPH::Shape* p_shape, uint64_t p_user_data) {
	JoltCustomUserDataShapeSettings shape_settings(p_shape);
	shape_settings.mUserData = (JPH::uint64)p_user_data;

	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Failed to override user data. "
			"It returned the following error: '%s'.",
			to_godot(shape_result.GetError())
		)
	);

	return shape_result.Get();
}

void JoltShapeImpl3D::_invalidated(bool p_lock) {
	for (const auto& [owner, ref_count] : ref_counts_by_owner) {
		owner->_shapes_changed(p_lock);
	}
}

String JoltShapeImpl3D::_owners_to_string() const {
	const int32_t owner_count = ref_counts_by_owner.size();

	if (owner_count == 0) {
		return "'<unknown>' and 0 other object(s)";
	}

	const JoltObjectImpl3D& random_owner = *ref_counts_by_owner.begin()->first;

	return vformat("'%s' and %d other object(s)", random_owner.to_string(), owner_count - 1);
}
