# ![Godot Jolt][log]

Godot Jolt is a native extension for the [Godot game engine][god] (4.0 or newer) that allows you to
use the [Jolt physics engine][jlt] to power Godot's 3D physics.

It functions as a drop-in replacement for Godot Physics, by implementing the same nodes that you
would use normally, like `RigidBody3D` or `CharacterBody3D`.

## Table of Contents

- [What features are there?](#what-features-are-there)
- [What about determinism?](#what-about-determinism)
- [What's missing?](#whats-missing)
- [What's different?](#whats-different)
- [What platforms are supported?](#what-platforms-are-supported)
- [How much does it cost?](#how-much-does-it-cost)
- [How do I get started?](#how-do-i-get-started)
- [What's planned for the future?](#whats-planned-for-the-future)
- [What do the project settings do?](#what-do-the-project-settings-do)
- [How do I build it from source?](#how-do-i-build-it-from-source)
- [What do the version numbers mean?](#what-do-the-version-numbers-mean)
- [How is it licensed?](#how-is-it-licensed)

## What features are there?

Improved performance, mainly. At the moment Godot Jolt is purely a drop-in replacement with no
additional nodes or features. This does however come with the benefit of allowing you to quickly
switch between physics engines with very little risk or effort.

## What about determinism?

While Jolt itself offers deterministic simulations, Godot Jolt is not able to make those kinds of
guarantees. There is a project setting for enabling Jolt's determinism if you still find yourself
wanting more deterministic behavior, but it should not be relied upon if determinism is a
requirement.

## What's missing?

- The standard joints do not support any sort of springs or soft limits
- Custom integrators are not supported
- `SoftBody3D` is not supported
- `WorldBoundaryShape3D` is not supported
- `StaticBody3D` does not support "constant velocities"
- The physics server is not thread-safe
- Memory usage is not reflected in Godot's performance monitors

## What's different?

- `HeightMapShape3D` only supports square height maps with dimensions that are power-of-two
- Collision layers/masks behave like they did in Godot 3, meaning you can't have one-way collisions
- Ray-casts will hit the back-faces of all shape types, not just concave polygons and height maps
- Shape-casts should be more accurate, but their cost also scale with the cast distance
- Axis-locking is implemented using joints, which means a body can technically deviate a bit from
  its locked axes, but they do a better job of preserving energy

## What platforms are supported?

- Windows (x86-64, x86)
- Linux (x86-64, x86)
- macOS (x86-64 + Apple Silicon)

## How much does it cost?

Much like Godot itself, Godot Jolt is free, open-source, permissively licensed and comes with no
expectations of payment, but if you'd still like to donate as a token of appreciation, then you're
welcome to do so through [GitHub Sponsors][spn].

## How do I get started?

1. Download the [latest release][rls] from GitHub
2. Extract the files to your project directory
3. Start (or restart) Godot
4. Open your project settings
5. Make sure "Advanced Settings" is enabled
6. Go to "Physics" and then "3D"
7. Change "Physics Engine" to "JoltPhysics3D"
8. Restart Godot

## What's planned for the future?

Without making any promises, and in no particular order, here are some of the bigger things:

- Adding substitutes for the joints, allowing for things like springs
- Adding new types of joints, like Jolt's `DistanceConstraint`
- Adding support for double-precision, allowing for large worlds
- Adding support for iOS and Android
- Making the physics server thread-safe

See the [`v1.0.0`][prj] project board for a more up-to-date view of the plan.

## What do the project settings do?

See [`docs/settings.md`][set] for information about the project settings that Godot Jolt offers.

## How do I build it from source?

See [`docs/building.md`][bld] for information about how to build Godot Jolt from source.

## What do the version numbers mean?

Godot Jolt uses [semantic versioning][smv], formatted as `<major>.<minor>.<patch>`. The major
version will be incremented when backwards-incompatible API changes are made, the minor version will
be incremented when backwards-compatible API changes are made and the patch version will be
incremented when changes are made that don't affect the API.

Note that major version `0.x.y` carries a special meaning in semantic versioning, where even minor
versions may contain backwards-incompatible changes.

See [`CHANGELOG.md`][chl] for details about what changes were included in each release.

## How is it licensed?

Godot Jolt is distributed under the MIT license. See [`LICENSE.txt`][lic] for more details and
[`THIRDPARTY.txt`][trd] for third-party licenses.

[log]: docs/logo.svg
[god]: https://godotengine.org/
[jlt]: https://github.com/jrouwe/JoltPhysics
[spn]: https://github.com/sponsors/mihe
[rls]: https://github.com/godot-jolt/godot-jolt/releases/latest
[prj]: https://github.com/orgs/godot-jolt/projects/1
[set]: docs/settings.md
[bld]: docs/building.md
[smv]: https://semver.org/
[chl]: CHANGELOG.md
[lic]: LICENSE.txt
[trd]: THIRDPARTY.txt
