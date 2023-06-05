# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Breaking changes are denoted with ⚠️.

## [Unreleased]

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

[Unreleased]: https://github.com/godot-jolt/godot-jolt/compare/v0.1.0-stable...HEAD
[0.1.0]: https://github.com/godot-jolt/godot-jolt/releases/tag/v0.1.0-stable
