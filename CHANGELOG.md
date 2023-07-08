# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Breaking changes are denoted with ⚠️.

## [Unreleased]

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

[Unreleased]: https://github.com/godot-jolt/godot-jolt/compare/v0.4.1-stable...HEAD
[0.4.1]: https://github.com/godot-jolt/godot-jolt/compare/v0.4.0-stable...v0.4.1-stable
[0.4.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.3.0-stable...v0.4.0-stable
[0.3.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.3-stable...v0.3.0-stable
[0.2.3]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.2-stable...v0.2.3-stable
[0.2.2]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.1-stable...v0.2.2-stable
[0.2.1]: https://github.com/godot-jolt/godot-jolt/compare/v0.2.0-stable...v0.2.1-stable
[0.2.0]: https://github.com/godot-jolt/godot-jolt/compare/v0.1.0-stable...v0.2.0-stable
[0.1.0]: https://github.com/godot-jolt/godot-jolt/releases/tag/v0.1.0-stable
