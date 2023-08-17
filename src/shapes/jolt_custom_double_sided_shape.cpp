#include "jolt_custom_double_sided_shape.hpp"

namespace {

JPH::Shape* construct_double_sided() {
	return new JoltCustomDoubleSidedShape();
}

void collide_shape_vs_double_sided(
	const JPH::Shape* p_shape1,
	const JPH::Shape* p_shape2,
	JPH::Vec3Arg p_scale1,
	JPH::Vec3Arg p_scale2,
	JPH::Mat44Arg p_center_of_mass_transform1,
	JPH::Mat44Arg p_center_of_mass_transform2,
	const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	const JPH::CollideShapeSettings& p_collide_shape_settings,
	JPH::CollideShapeCollector& p_collector,
	const JPH::ShapeFilter& p_shape_filter
) {
	ERR_FAIL_COND(p_shape2->GetSubType() != JoltCustomShapeSubType::DOUBLE_SIDED);

	const auto* shape2 = static_cast<const JoltCustomDoubleSidedShape*>(p_shape2);

	JPH::CollideShapeSettings new_collide_shape_settings = p_collide_shape_settings;
	new_collide_shape_settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

	JPH::CollisionDispatch::sCollideShapeVsShape(
		p_shape1,
		shape2->GetInnerShape(),
		p_scale1,
		p_scale2,
		p_center_of_mass_transform1,
		p_center_of_mass_transform2,
		p_sub_shape_id_creator1,
		p_sub_shape_id_creator2,
		new_collide_shape_settings,
		p_collector,
		p_shape_filter
	);
}

void cast_shape_vs_double_sided(
	const JPH::ShapeCast& p_shape_cast,
	const JPH::ShapeCastSettings& p_shape_cast_settings,
	const JPH::Shape* p_shape,
	JPH::Vec3Arg p_scale,
	const JPH::ShapeFilter& p_shape_filter,
	JPH::Mat44Arg p_center_of_mass_transform2,
	const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	JPH::CastShapeCollector& p_collector
) {
	ERR_FAIL_COND(p_shape->GetSubType() != JoltCustomShapeSubType::DOUBLE_SIDED);

	const auto* shape = static_cast<const JoltCustomDoubleSidedShape*>(p_shape);

	JPH::ShapeCastSettings new_shape_cast_settings = p_shape_cast_settings;
	new_shape_cast_settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;

	JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(
		p_shape_cast,
		new_shape_cast_settings,
		shape->GetInnerShape(),
		p_scale,
		p_shape_filter,
		p_center_of_mass_transform2,
		p_sub_shape_id_creator1,
		p_sub_shape_id_creator2,
		p_collector
	);
}

} // namespace

JPH::ShapeSettings::ShapeResult JoltCustomDoubleSidedShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		new JoltCustomDoubleSidedShape(*this, mCachedResult);
	}

	return mCachedResult;
}

void JoltCustomDoubleSidedShape::register_type() {
	JPH::ShapeFunctions& shape_functions = JPH::ShapeFunctions::sGet(
		JoltCustomShapeSubType::DOUBLE_SIDED
	);

	shape_functions.mConstruct = construct_double_sided;
	shape_functions.mColor = JPH::Color::sPurple;

	for (const JPH::EShapeSubType sub_type : JPH::sAllSubShapeTypes) {
		JPH::CollisionDispatch::sRegisterCollideShape(
			sub_type,
			JoltCustomShapeSubType::DOUBLE_SIDED,
			collide_shape_vs_double_sided
		);

		JPH::CollisionDispatch::sRegisterCastShape(
			sub_type,
			JoltCustomShapeSubType::DOUBLE_SIDED,
			cast_shape_vs_double_sided
		);
	}
}
