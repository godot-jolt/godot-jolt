# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Changed the direction of `Generic6DOFJoint` angular motors to match Godot Physics
- Changed shape margins to be interpreted as an upper bound. They are now scaled according to the
  shape's size, which removes the ability to have an incorrect margin, thereby removing any warnings
  about that.

### Added

- Added support for custom integrators
- Added new project setting "Bounce Velocity Threshold"
- Added support for "Rough" physics material property
- Added support for "Absorbent" physics material property
- Added support for surface velocities, also known as "constant velocities", for both static and
  kinematic bodies
- Added support for more flexible limits for `HingeJoint3D` and `SliderJoint3D`

### Fixed

- Fixed issue where friction values weren't combined in the same way as Godot Physics
- Fixed issue where bounce values weren't combined in the same way as Godot Physics
- Fixed issue where setting friction on an already entered body would instead set bounce
- Fixed issue where setting velocities on kinematic bodies would actually move them instead of
  applying a surface velocity
- Fixed issue where `RigidBody3D` would de-sync from its actual physics body after freezing
- Fixed issues where disabling or removing a body connected to a joint would error or crash
- Fixed issue with `CharacterBody3D` and other kinematic bodies where they wouldn't elicit a proper
  collision response from dynamic bodies

## [0.1.0] - 2023-05-24

Initial release.

[Unreleased]: https://github.com/godot-jolt/godot-jolt/compare/v0.1.0-stable...HEAD
[0.1.0]: https://github.com/godot-jolt/godot-jolt/releases/tag/v0.1.0-stable
