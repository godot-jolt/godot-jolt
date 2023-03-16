#pragma once

class JoltPhysicsServer3D;
class JoltBody3D;

class JoltMotionFilter3D final
	: public JPH::BroadPhaseLayerFilter
	, public JPH::ObjectLayerFilter
	, public JPH::BodyFilter
	, public JPH::ShapeFilter {
public:
	JoltMotionFilter3D(const JoltBody3D& p_body, bool p_collide_separation_ray);

	bool ShouldCollide(JPH::BroadPhaseLayer p_broad_phase_layer) const override;

	bool ShouldCollide(JPH::ObjectLayer p_object_layer) const override;

	bool ShouldCollide(const JPH::BodyID& p_body_id) const override;

	bool ShouldCollideLocked(const JPH::Body& p_body) const override;

	bool ShouldCollide(const JPH::Shape* p_shape2, const JPH::SubShapeID& p_sub_shape_id2)
		const override;

	bool ShouldCollide(
		const JPH::Shape* p_shape1,
		const JPH::SubShapeID& p_sub_shape_id1,
		const JPH::Shape* p_shape2,
		const JPH::SubShapeID& p_sub_shape_id2
	) const override;

private:
	const JoltPhysicsServer3D& physics_server;

	const JoltBody3D& body;

	bool collide_separation_ray = false;
};
