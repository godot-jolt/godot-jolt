#include "jolt_empty_shape.hpp"

namespace {

JPH::Shape* construct_empty() {
	return new JoltEmptyShape();
}

void collide_noop(
	[[maybe_unused]] const JPH::Shape* p_shape1,
	[[maybe_unused]] const JPH::Shape* p_shape2,
	[[maybe_unused]] JPH::Vec3Arg p_scale1,
	[[maybe_unused]] JPH::Vec3Arg p_scale2,
	[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform1,
	[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform2,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	[[maybe_unused]] const JPH::CollideShapeSettings& p_collide_shape_settings,
	[[maybe_unused]] JPH::CollideShapeCollector& p_collector,
	[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter
) { }

void cast_noop(
	[[maybe_unused]] const JPH::ShapeCast& p_shape_cast,
	[[maybe_unused]] const JPH::ShapeCastSettings& p_shape_cast_settings,
	[[maybe_unused]] const JPH::Shape* p_shape,
	[[maybe_unused]] JPH::Vec3Arg p_scale,
	[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter,
	[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform2,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	[[maybe_unused]] JPH::CastShapeCollector& p_collector
) { }

} // namespace

JPH::ShapeSettings::ShapeResult JoltEmptyShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		new JoltEmptyShape(*this, mCachedResult);
	}

	return mCachedResult;
}

void JoltEmptyShape::register_type() {
	JPH::ShapeFunctions& shape_functions = JPH::ShapeFunctions::sGet(JPH::EShapeSubType::User1);

	shape_functions.mConstruct = construct_empty;
	shape_functions.mColor = JPH::Color::sBlack;

	for (const JPH::EShapeSubType sub_type : JPH::sAllSubShapeTypes) {
		JPH::CollisionDispatch::sRegisterCollideShape(
			JPH::EShapeSubType::User1,
			sub_type,
			collide_noop
		);

		JPH::CollisionDispatch::sRegisterCollideShape(
			sub_type,
			JPH::EShapeSubType::User1,
			collide_noop
		);

		JPH::CollisionDispatch::sRegisterCastShape(JPH::EShapeSubType::User1, sub_type, cast_noop);

		JPH::CollisionDispatch::sRegisterCastShape(sub_type, JPH::EShapeSubType::User1, cast_noop);
	}
}
