# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Changed the direction of `Generic6DOFJoint` angular motors to match Godot Physics

### Added

- Added support for custom integrators
- Added new project setting "Bounce Velocity Threshold"
- Added support for "Rough" physics material property
- Added support for "Absorbent" physics material property

### Fixed

- Fixed issue where friction values weren't combined in the same way as Godot Physics
- Fixed issue where bounce values weren't combined in the same way as Godot Physics
- Fixed issue where setting friction on an already entered body would instead set bounce

## [0.1.0] - 2023-05-24

Initial release.

[Unreleased]: https://github.com/godot-jolt/godot-jolt/compare/v0.1.0-stable...HEAD
[0.1.0]: https://github.com/godot-jolt/godot-jolt/releases/tag/v0.1.0-stable
