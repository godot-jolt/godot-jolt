#include "jolt_ray_shape.hpp"

#include "jolt_query_collectors.hpp"

namespace {

class JoltRayConvexSupport final : public JPH::ConvexShape::Support {
public:
	explicit JoltRayConvexSupport(float p_length)
		: length(p_length) { }

	JPH::Vec3 GetSupport(JPH::Vec3Arg p_direction) const override {
		if (p_direction.GetZ() > 0.0f) {
			return {0.0f, 0.0f, length};
		} else {
			return JPH::Vec3::sZero();
		}
	}

	float GetConvexRadius() const override { return 0.0f; }

private:
	float length = 0.0f;
};

static_assert(sizeof(JoltRayConvexSupport) <= sizeof(JPH::ConvexShape::SupportBuffer));

JPH::Shape* construct_ray() {
	return new JoltRayShape();
}

void collide_ray_vs_shape(
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
	[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter
) {
	const auto* shape1 = static_cast<const JoltRayShape*>(p_shape1);

	// TODO(mihe): This transform scale/inverse feels unnecessary and should be optimized

	const JPH::Mat44 transform1 = p_center_of_mass_transform1 * JPH::Mat44::sScale(p_scale1);
	const JPH::Mat44 transform2 = p_center_of_mass_transform2 * JPH::Mat44::sScale(p_scale2);
	const JPH::Mat44 transform_inv2 = transform2.Inversed();

	const JPH::Vec3 start = transform1.GetTranslation();
	const JPH::Vec3 direction = transform1.GetAxisZ();
	const JPH::Vec3 vector = direction * shape1->length;

	const JPH::Vec3 local_start2 = transform_inv2 * start;
	const JPH::Vec3 local_direction2 = transform_inv2.Multiply3x3(direction);
	const JPH::Vec3 local_vector2 = transform_inv2.Multiply3x3(vector);

	const JPH::RayCast ray_cast(local_start2, local_vector2);

	JPH::RayCastSettings ray_cast_settings;
	ray_cast_settings.mBackFaceMode = p_collide_shape_settings.mBackFaceMode;

	// NOTE(mihe): Treating convex shapes as solid deviates from how these shapes behave in Godot
	// Physics, where no collision is registered when fully enveloped by a convex shape. It's not
	// clear to me whether the behavior of Godot Physics is desired or just the result of a simple
	// implementation. I assume it's the latter, hence the deviation.
	ray_cast_settings.mTreatConvexAsSolid = true;

	JoltQueryCollectorClosest<JPH::CastRayCollector> collector;

	p_shape2->CastRay(ray_cast, ray_cast_settings, p_sub_shape_id_creator2, collector);

	if (!collector.had_hit()) {
		return;
	}

	const JPH::RayCastResult& hit = collector.get_hit();

	const JPH::Vec3 local_point2 = local_start2 + local_vector2 * hit.mFraction;
	const JPH::Vec3 local_normal2 = shape1->slide_on_slope
		? p_shape2->GetSurfaceNormal(hit.mSubShapeID2, local_point2)
		: -local_direction2;

	const JPH::Vec3 point_on_1 = start + vector;
	const JPH::Vec3 point_on_2 = transform2 * local_point2;
	const JPH::Vec3 normal = transform2.Multiply3x3(local_normal2);

	const JPH::CollideShapeResult result(
		point_on_1,
		point_on_2,
		-normal,
		(point_on_1 - point_on_2).Length(),
		p_sub_shape_id_creator1.GetID(),
		p_sub_shape_id_creator2.GetID(),
		JPH::TransformedShape::sGetBodyID(p_collector.GetContext())
	);

	// TODO(mihe): Implement face collecting

	p_collector.AddHit(result);
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

JPH::ShapeSettings::ShapeResult JoltRayShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		new JoltRayShape(*this, mCachedResult);
	}

	return mCachedResult;
}

void JoltRayShape::register_type() {
	JPH::ShapeFunctions& shape_functions =
		JPH::ShapeFunctions::sGet(JPH::EShapeSubType::UserConvex1);

	shape_functions.mConstruct = construct_ray;
	shape_functions.mColor = JPH::Color::sDarkRed;

	static constexpr JPH::EShapeSubType concrete_sub_types[] = {
		JPH::EShapeSubType::Sphere,
		JPH::EShapeSubType::Box,
		JPH::EShapeSubType::Triangle,
		JPH::EShapeSubType::Capsule,
		JPH::EShapeSubType::TaperedCapsule,
		JPH::EShapeSubType::Cylinder,
		JPH::EShapeSubType::ConvexHull,
		JPH::EShapeSubType::Mesh,
		JPH::EShapeSubType::HeightField};

	for (const JPH::EShapeSubType concrete_sub_type : concrete_sub_types) {
		JPH::CollisionDispatch::sRegisterCollideShape(
			JPH::EShapeSubType::UserConvex1,
			concrete_sub_type,
			collide_ray_vs_shape
		);

		JPH::CollisionDispatch::sRegisterCollideShape(
			concrete_sub_type,
			JPH::EShapeSubType::UserConvex1,
			JPH::CollisionDispatch::sReversedCollideShape
		);
	}

	JPH::CollisionDispatch::sRegisterCollideShape(
		JPH::EShapeSubType::UserConvex1,
		JPH::EShapeSubType::UserConvex1,
		collide_noop
	);

	JPH::CollisionDispatch::sRegisterCastShape(
		JPH::EShapeSubType::UserConvex1,
		JPH::EShapeSubType::UserConvex1,
		cast_noop
	);
}

JPH::AABox JoltRayShape::GetLocalBounds() const {
	return {JPH::Vec3::sZero(), JPH::Vec3(0.0f, 0.0f, length)};
}

#ifdef JPH_DEBUG_RENDERER

void JoltRayShape::Draw(
	JPH::DebugRenderer* p_renderer,
	JPH::RMat44Arg p_center_of_mass_transform,
	JPH::Vec3Arg p_scale,
	JPH::ColorArg p_color,
	bool p_use_material_colors,
	[[maybe_unused]] bool p_draw_wireframe
) const {
	p_renderer->DrawArrow(
		p_center_of_mass_transform.GetTranslation(),
		p_center_of_mass_transform * JPH::Vec3(0, 0, length * p_scale.GetZ()),
		p_use_material_colors ? GetMaterial()->GetDebugColor() : p_color,
		0.1f
	);
}

#endif // JPH_DEBUG_RENDERER

const JPH::ConvexShape::Support* JoltRayShape::GetSupportFunction(
	[[maybe_unused]] JPH::ConvexShape::ESupportMode p_mode,
	JPH::ConvexShape::SupportBuffer& p_buffer,
	JPH::Vec3Arg p_scale
) const {
	return new (&p_buffer) JoltRayConvexSupport(p_scale.GetZ() * length);
}
