#include "jolt_physics_space_3d.hpp"

#include "conversion.hpp"
#include "error_macros.hpp"
#include "jolt_physics_collision_object_3d.hpp"
#include "jolt_physics_shape_3d.hpp"

namespace {

enum JoltPhysicsObjectLayer3D {
	GDJOLT_OBJECT_LAYER_STATIC,
	GDJOLT_OBJECT_LAYER_MOVING,
	GDJOLT_OBJECT_LAYER_COUNT
};

enum JoltPhysicsBroadPhaseLayer3D {
	GDJOLT_BROAD_PHASE_LAYER_STATIC,
	GDJOLT_BROAD_PHASE_LAYER_MOVING,
	GDJOLT_BROAD_PHASE_LAYER_COUNT
};

constexpr uint32_t GDJOLT_MAX_BODIES = 65536;
constexpr uint32_t GDJOLT_NUM_BODY_MUTEXES = 0; // 0 = default
constexpr uint32_t GDJOLT_MAX_BODY_PAIRS = 65536;
constexpr uint32_t GDJOLT_MAX_CONTACT_CONSTRAINTS = 65536;

bool jolt_can_collide_object(JPH::ObjectLayer p_object1, JPH::ObjectLayer p_object2) {
	switch (p_object1) {
	case GDJOLT_OBJECT_LAYER_STATIC:
		return p_object2 == GDJOLT_OBJECT_LAYER_MOVING;
	case GDJOLT_OBJECT_LAYER_MOVING:
		return true;
	default:
		ERR_FAIL_V_NOT_IMPL({});
	}
}

bool jolt_can_collide_broadphase(JPH::ObjectLayer p_layer1, JPH::BroadPhaseLayer p_layer2) {
	switch (p_layer1) {
	case GDJOLT_OBJECT_LAYER_STATIC:
		return p_layer2 == JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_MOVING);
	case GDJOLT_OBJECT_LAYER_MOVING:
		return true;
	default:
		ERR_FAIL_V_NOT_IMPL({});
	}
}

class JoltPhysicsLayerMapper3D final : public JPH::BroadPhaseLayerInterface {
public:
	JoltPhysicsLayerMapper3D() {
		broadphase_by_object[GDJOLT_OBJECT_LAYER_STATIC] =
			JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_STATIC);
		broadphase_by_object[GDJOLT_OBJECT_LAYER_MOVING] =
			JPH::BroadPhaseLayer(GDJOLT_BROAD_PHASE_LAYER_MOVING);
	}

	uint32_t GetNumBroadPhaseLayers() const override { return GDJOLT_BROAD_PHASE_LAYER_COUNT; }

	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer p_layer) const override {
		return broadphase_by_object[p_layer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer p_layer) const override {
		switch ((JPH::BroadPhaseLayer::Type)p_layer) {
		case GDJOLT_BROAD_PHASE_LAYER_STATIC:
			return "STATIC";
		case GDJOLT_BROAD_PHASE_LAYER_MOVING:
			return "MOVING";
		default:
			return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	JPH::BroadPhaseLayer broadphase_by_object[GDJOLT_OBJECT_LAYER_COUNT];
};

} // namespace

JoltPhysicsSpace3D::JoltPhysicsSpace3D(
	JPH::JobSystem* p_job_system,
	JPH::GroupFilter* p_group_filter
)
	: job_system(p_job_system)
	, temp_allocator(new JPH::TempAllocatorMalloc())
	, bp_layer(new JoltPhysicsLayerMapper3D())
	, physics_system(new JPH::PhysicsSystem())
	, group_filter(p_group_filter) {
	physics_system->Init(
		GDJOLT_MAX_BODIES,
		GDJOLT_NUM_BODY_MUTEXES,
		GDJOLT_MAX_BODY_PAIRS,
		GDJOLT_MAX_CONTACT_CONSTRAINTS,
		*bp_layer,
		jolt_can_collide_broadphase,
		jolt_can_collide_object
	);
}

JoltPhysicsSpace3D::~JoltPhysicsSpace3D() {
	delete physics_system;
	delete bp_layer;
	delete temp_allocator;
}

void JoltPhysicsSpace3D::step(float p_step) {
	lock();

	// TODO(mihe): Integrate forces/velocities

	physics_system->Update(p_step, 1, 1, temp_allocator, job_system);

	unlock();
}

void JoltPhysicsSpace3D::call_queries() {
	JPH::BodyIDVector body_ids;
	physics_system->GetActiveBodies(body_ids);

	{
		const JPH::BodyLockInterface& lock_iface = physics_system->GetBodyLockInterface();
		const JPH::BodyLockMultiRead lock(lock_iface, body_ids.data(), (int)body_ids.size());

		// TODO(mihe): Is the separation of bodies and areas here important? Maybe merge into a
		// single loop?

		for (int i = 0; i < (int)body_ids.size(); ++i) {
			if (const JPH::Body* body = lock.GetBody(i)) {
				if (!body->IsSensor()) {
					auto* object =
						reinterpret_cast<JoltPhysicsCollisionObject3D*>(body->GetUserData());
					object->call_queries();
				}
			}
		}

		for (int i = 0; i < (int)body_ids.size(); ++i) {
			if (const JPH::Body* body = lock.GetBody(i)) {
				if (body->IsSensor()) {
					auto* object =
						reinterpret_cast<JoltPhysicsCollisionObject3D*>(body->GetUserData());
					object->call_queries();
				}
			}
		}
	}
}

PhysicsDirectSpaceState3D* JoltPhysicsSpace3D::get_direct_state() const {
	return direct_state;
}

Variant JoltPhysicsSpace3D::get_param(PhysicsServer3D::AreaParameter p_param) const {
	switch (p_param) {
	case PhysicsServer3D::AREA_PARAM_GRAVITY:
		return gravity;
	case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
		return gravity_vector;
	default:
		ERR_FAIL_V_NOT_IMPL({});
	}
}

void JoltPhysicsSpace3D::set_param(PhysicsServer3D::AreaParameter p_param, const Variant& p_value) {
	switch (p_param) {
	case PhysicsServer3D::AREA_PARAM_GRAVITY:
		gravity = p_value;
		update_gravity();
		break;
	case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
		gravity_vector = p_value;
		update_gravity();
		break;
	default:
		ERR_FAIL_NOT_IMPL();
	}
}

void JoltPhysicsSpace3D::create_object(JoltPhysicsCollisionObject3D* p_object) {
	if (!p_object->get_jid().IsInvalid()) {
		return;
	}

	JPH::Ref<JPH::MutableCompoundShapeSettings> compound_shape =
		new JPH::MutableCompoundShapeSettings();

	for (const JoltPhysicsCollisionObject3D::Shape& shape : p_object->get_shapes()) {
		compound_shape->AddShape(
			to_jolt(shape.transform.origin),
			to_jolt(shape.transform.basis),
			shape.ref->get_jref()
		);
	}

	JPH::EMotionType motion_type = {};
	JPH::ObjectLayer object_layer = {};

	switch (p_object->get_mode()) {
	case PhysicsServer3D::BODY_MODE_STATIC:
		motion_type = JPH::EMotionType::Static;
		object_layer = GDJOLT_OBJECT_LAYER_STATIC;
		break;
	case PhysicsServer3D::BODY_MODE_KINEMATIC:
		motion_type = JPH::EMotionType::Kinematic;
		object_layer = GDJOLT_OBJECT_LAYER_MOVING;
		break;
	case PhysicsServer3D::BODY_MODE_RIGID:
	case PhysicsServer3D::BODY_MODE_RIGID_LINEAR:
		motion_type = JPH::EMotionType::Dynamic;
		object_layer = GDJOLT_OBJECT_LAYER_MOVING;
		break;
	default:
		ERR_FAIL_MSG("Unhandled body mode");
	}

	const Transform3D& transform = p_object->get_transform();
	const bool is_sensor = p_object->is_sensor();

	JPH::BodyCreationSettings settings(
		compound_shape,
		to_jolt(transform.origin),
		to_jolt(transform.basis),
		motion_type,
		object_layer
	);

	settings.mAllowDynamicOrKinematic = true;
	settings.mIsSensor = is_sensor;
	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
	settings.mMassPropertiesOverride.mMass = p_object->get_mass();

	JPH::Body* body = physics_system->GetBodyInterface().CreateBody(settings);

	body->SetCollisionGroup(JPH::CollisionGroup(
		group_filter,
		p_object->get_collision_layer(),
		p_object->get_collision_mask()
	));

	body->SetUserData(reinterpret_cast<JPH::uint64>(p_object));

	p_object->set_jid(body->GetID());
}

void JoltPhysicsSpace3D::add_object(JoltPhysicsCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().AddBody(p_object->get_jid(), JPH::EActivation::Activate);
}

void JoltPhysicsSpace3D::remove_object(JoltPhysicsCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().RemoveBody(p_object->get_jid());
}

void JoltPhysicsSpace3D::destroy_object(JoltPhysicsCollisionObject3D* p_object) {
	physics_system->GetBodyInterface().DestroyBody(p_object->get_jid());
	p_object->set_jid(JPH::BodyID());
}

void JoltPhysicsSpace3D::update_gravity() {
	physics_system->SetGravity(to_jolt(gravity_vector * gravity));
}
