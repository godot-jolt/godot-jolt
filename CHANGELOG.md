# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Breaking changes are denoted with ⚠️.

## [Unreleased]

### Changed

- ⚠️ Changed the `body_set_force_integration_callback` method of `PhysicsServer3D` to behave like it
  does with Godot Physics, where omitting the binding of `userdata` requires that the callback also
  doesn't take any `userdata`.

### Added

- Added timings of Jolt's various jobs to the "Physics 3D" profiler category.

### Fixed

- Fixed issue with `move_and_slide`, where under certain conditions you could get stuck on internal
  edges of a `ConcavePolygonShape3D` if the floor was within 5-ish degrees of `floor_max_angle`.
- Fixed issue with `move_and_slide`, where under certain conditions, while using a `BoxShape3D` or
  `CylinderShape3D` shape, you could get stuck on internal edges of a `ConcavePolygonShape3D`.
- Fixed issue where collision with `ConvexPolygonShape3D` could yield a flipped contact normal.

## [0.7.0] - 2023-08-29

### Removed

- ⚠️ Disabled the `JoltDebugGeometry3D` node for all distributed builds. If you still need it, build
  from source using the `*-development` or `*-debug` configurations.

### Changed

- ⚠️ Ray-casts will no longer hit the back-faces of `ConcavePolygonShape3D` if its `hit_back_faces`
  parameter is set to `false`, regardless of what the `backface_collision` property of the
  `ConcavePolygonShape3D` is set to.
- ⚠️ Changed the triangulation of `HeightMapShape3D` to match Godot Physics.

### Fixed

- ⚠️ Fixed regression where a motored `HingeJoint3D` (or `JoltHingeJoint3D`) would rotate
  counter-clockwise instead of clockwise.
- Fixed issue where ray-casting a `ConcavePolygonShape3D` that had `backface_collision` enabled, you
  would sometimes end up with a flipped normal.
- Fixed issue where a `CharacterBody3D` using `move_and_slide` could sometimes get stuck when
  sliding along a wall.
- Fixed issue where attaching a `RigidBody3D` with locked axes to a joint could result in NaN
  velocities/position and subsequently a lot of random errors being emitted from within Godot.

## [0.6.0] - 2023-08-17

### Changed

- Changed the editor gizmo for `JoltPinJoint3D`.
- Changed the editor gizmo for `JoltHingeJoint3D`.
- Changed the editor gizmo for `JoltSliderJoint3D`.
- Changed the editor gizmo for `JoltConeTwistJoint3D`.
- Changed the editor gizmo for `JoltGeneric6DOFJoint3D`.

### Added

- Added support for `HeightMapShape3D` with non-power-of-two dimensions.
- Added support for `HeightMapShape3D` with non-square dimensions.
- Added support for `HeightMapShape3D` with no heights.

### Fixed

- Fixed issue where bodies would catch on internal edges of `ConcavePolygonShape3D`.

## [0.5.0] - 2023-08-08

### Removed

- ⚠️ Removed the ability to lock all six axes of a `RigidBody3D`. Consider freezing the body as
  static instead.

### Added

- Added substitutes for all the joint nodes, to better align with the interface that Jolt offers,
  which consist of `JoltPinJoint3D`, `JoltHingeJoint3D`, `JoltSliderJoint3D`, `JoltConeTwistJoint3D`
  and `JoltGeneric6DOFJoint3D`. These differ in the following ways:
  - You can enable/disable the limits on all joints.
  - You can enable/disable the joint itself using its `enabled` property.
  - You can fetch the magnitude of the force/torque that was last applied to keep the joint
    together, using the `get_applied_force` and `get_applied_torque` methods. These coupled with the
    `enabled` property allows for creating breakable joints.
  - You can increase the joint's solver iterations, to improve stability, using its
    `solver_velocity_iterations` and `solver_position_iterations` properties.
  - Springs use frequency and damping instead of stiffness and damping.
  - Soft limits are achieved with limit springs.
  - `JoltConeTwistJoint3D` can be configured with a motor.
  - Angular motor velocities are set in radians per second, but displayed in degrees per second.
  - Any motion parameters like bias, damping and relaxation are omitted.
  - Any angular motion parameters for the slider joint are omitted.

### Fixed

- Fixed issue where linear axis locks could be budged a bit if enough force was applied.
- Fixed issue where `CharacterBody3D` and other kinematic bodies wouldn't respect locked axes.
- Fixed issue where passing `null` to the `result` parameter (or omitting it entirely) of the
  `body_test_motion` method in `PhysicsServer3D` would cause a crash.
- Fixed issue where the `body_is_omitting_force_integration` method in `PhysicsServer3D` would
  always return `false`.

## [0.4.1] - 2023-07-08

### Fixed

- Fixed issue where colliding with certain types of degenerate triangles in `ConcavePolygonShape3D`
  would cause the application to hang or emit a vast amount of errors.

## [0.4.0] - 2023-07-08

### Changed

- ⚠️ Changed the `cast_motion` method in `PhysicsDirectSpaceState3D` to return `[1.0, 1.0]` instead
  of `[]` when no collision was detected, to match Godot Physics.
- ⚠️ Changed contact positions to be absolute global positions instead of relative global positions,
  to match the new behavior in Godot Physics.

### Added

- Added support for springs in `Generic6DOFJoint3D`.

### Fixed

- Fixed issue where angular surface velocities (like `constant_angular_velocity` on `StaticBody3D`)
  wouldn't be applied as expected if the imparted upon body was placed across the imparting body's
  center of mass.
- Fixed issue where going from `CENTER_OF_MASS_MODE_CUSTOM` to `CENTER_OF_MASS_MODE_AUTO` wouldn't
  actually reset the body's center-of-mass.
- Fixed issue where any usage of `PhysicsServer3D`, `PhysicsDirectBodyState3D` or
  `PhysicsDirectSpaceState3D` in C# scripts would trigger an exception.
- Fixed issue where the `recovery_as_collision` parameter in the `move_and_collide` and `test_move`
  methods on bodies would always be `true`.
- Fixed issue where the `input_ray_pickable` property on bodies and areas would always be `true`.

## [0.3.0] - 2023-06-28

### Changed

- ⚠️ Changed collision layers and masks to behave as they do in Godot Physics, allowing for
  asymmetrical collisions, where the body whose mask does not contain the layer of the other body
  effectively gets infinite mass and inertia in the context of that collision.

### Added

- Added new project setting, "Use Shape Margins", which when disabled leads to all shape margins
  being ignored and instead set to 0, at a slight performance cost.
- Added new project setting, "Areas Detect Static Bodies", to allow `Area3D` to detect overlaps with
  static bodies (including `RigidBody3D` using `FREEZE_MODE_STATIC`) at a potentially heavy
  performance/memory cost.

### Fixed

- Fixed issue where a `RigidBody3D` using `FREEZE_MODE_KINEMATIC` wouldn't have its
  `_integrate_forces` method called when monitoring contacts.
- Fixed issue where scaling bodies/shapes with negative values would break them in various ways.
- Fixed issue where `CharacterBody3D` platform velocities would always be zero.
- Fixed issue where the velocity of kinematic colliders would always be zero in `_physics_process`.

## [0.2.3] - 2023-06-16

### Fixed

- Fixed issue where bodies would transform in unintuitive ways when attached to a rotated joint.
- Fixed issue where bodies would sometimes transform in unintuitive ways when attached to a
  `Generic6DOFJoint` that used both linear and angular limits.
- Fixed issue where setting the limits of a `SliderJoint3D` to the same value would make it free
  instead of fixed.
- Fixed issue where you could still rotate a `RigidBody3D` slightly while using `lock_rotation`.
- Fixed issue where friction would be applied more on one axis than the other.

## [0.2.2] - 2023-06-09

### Fixed

- Fixed issue where `AnimatableBody3D` would de-sync from its underlying body when moved.
- Fixed issue where `CharacterBody3D` and other kinematic bodies would sometimes maintain a velocity
  after having moved.

## [0.2.1] - 2023-06-06

### Fixed

- Fixed issue where having scaled bodies attached to a joint would result in the bodies being
  displaced from their starting position.

## [0.2.0] - 2023-06-06

### Changed

- ⚠️ Changed friction values to be combined in the same way as in Godot Physics.
- ⚠️ Changed bounce values to be combined in the same way as in Godot Physics.
- ⚠️ Changed the direction of `Generic6DOFJoint` angular motors to match Godot Physics.
- ⚠️ Changed how linear/angular velocities are applied to a frozen `RigidBody3D`, to better match
  Godot Physics. They now apply a surface velocity, also known as a "constant velocity", instead of
  actually moving the body.
- ⚠️ Changed shape margins to be interpreted as an upper bound. They are now scaled according to the
  shape's extents, which removes the ability to have an incorrect margin, thereby removing any
  warnings about that.
- Changed warning/error messages to provide more context, such as the names of any bodies
  related/connected to that particular thing.

### Added

- Added new project setting "Bounce Velocity Threshold".
- Added support for the `custom_integrator` property on `RigidBody3D` and `PhysicalBone3D`.
- Added support for the `integrate_forces` method on `PhysicsDirectBodyState3D`.
- Added support for the `rough` and `absorbent` properties on `PhysicsMaterial`.
- Added support for surface velocities, also known as "constant velocities", for both static and
  kinematic bodies.
- Added support for more flexible limits for `HingeJoint3D` and `SliderJoint3D`.

### Fixed

- Fixed issue where `CharacterBody3D` and other kinematic bodies wouldn't elicit a proper collision
  response from dynamic bodies.
- Fixed issue where setting friction on an already entered body would instead set bounce.
- Fixed issue where disabling or removing a body connected to a joint would error or crash.
- Fixed issue where `RigidBody3D` would de-sync from its underlying body after freezing with the
  "Static" freeze mode.
- Fixed issue where bodies connected to a `Generic6DOFJoint` would lose their relative pose when
  changing any of the joint's limits

## [0.1.0] - 2023-05-24

Initial release.

[Unreleased]: https://github.com/godot-jolt/godot-jolt/compare/v0.7.0-stable...HEAD
[0.7.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.6.0-stable...v0.7.0-stable
[0.6.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.5.0-stable...v0.6.0-stable
[0.5.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.4.1-stable...v0.5.0-stable
[0.4.1]: https://github.com/godot-jolt/godot-jolt/compare/v0.4.0-stable...v0.4.1-stable
[0.4.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.3.0-stable...v0.4.0-stable
[0.3.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.3-stable...v0.3.0-stable
[0.2.3]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.2-stable...v0.2.3-stable
[0.2.2]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.1-stable...v0.2.2-stable
[0.2.1]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.0-stable...v0.2.1-stable
[0.2.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.1.0-stable...v0.2.0-stable
[0.1.0]: https://github.com/godot-jolt/godot-jolt/releases/tag/v0.1.0-stable
