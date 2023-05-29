# ![Godot Jolt][log]

Godot Jolt is a native extension for the [Godot game engine][god] that allows you to use the [Jolt
physics engine][jlt] to power Godot's 3D physics.

It functions as a drop-in replacement for Godot Physics, by implementing the same nodes that you
would use normally, like `RigidBody3D` or `CharacterBody3D`.

## Table of Contents

- [What features are there?](#what-features-are-there)
- [What about determinism?](#what-about-determinism)
- [What's not supported?](#whats-not-supported)
- [What else is different?](#what-else-is-different)
- [What versions of Godot are supported?](#what-versions-of-godot-are-supported)
- [What platforms are supported?](#what-platforms-are-supported)
- [How do I get started?](#how-do-i-get-started)
- [What's the plan going forward?](#whats-the-plan-going-forward)
- [What settings are there?](#what-settings-are-there)
- [How do I build from source?](#how-do-i-build-from-source)
- [What do the versions mean?](#what-do-the-versions-mean)
- [What's the license?](#whats-the-license)

## What features are there?

Better performance, mainly, but also just having different characteristics compared to Godot
Physics.

At the moment Godot Jolt is purely a drop-in replacement with no additional nodes or features. This
allows you to quickly switch between physics engines with little risk or effort.

## What about determinism?

While Jolt itself offers deterministic simulations, Godot Jolt is not able to make such guarantees.
Simulations in Godot Jolt may look deterministic, and may even happen to be deterministic, but this
should not be relied upon if determinism is a hard requirement.

## What's not supported?

- Joints do not support springs or soft limits (yet)
- `SoftBody3D` is not supported
- `WorldBoundaryShape3D` is not supported
- The physics server is not thread-safe (yet)
- Memory usage is not reflected in Godot's performance monitors (yet)

## What else is different?

- Collision layers/masks behave like they did in Godot 3, meaning you can't have one-way collisions
- Ray-casts will hit the back-faces of all shape types, not just concave polygons and height maps
- Shape-casts should be more accurate, but their cost also scale with the cast distance
- Manipulating a body's shape(s) while it's connected to a scene tree can be costly
- Contact impulses are estimations and won't be accurate when colliding with multiple bodies
- `Area3D` is not able to detect overlaps with `StaticBody3D`
- `HeightMapShape3D` only supports square height maps with dimensions that are power-of-two
- Axis-locking is implemented using joints, which means a body can technically deviate a bit from
  its locked axes, but they do a better job of conserving energy

Also consider this note from Jolt's [documentation][jdc]:

> In order for the simulation to be accurate, dynamic objects should be in the order of 0.1 to 10
> meters long and have speeds in the order of 0 to 500 meters per second. Static object should be in
> the order of 0.1 to 2000 meters long.

## What versions of Godot are supported?

Godot Jolt relies on the GDExtension API found in Godot 4 and as a result only supports the latest
stable version of Godot 4. Any custom build or pre-release of Godot, such as a beta or release
candidate, may be incompatible and is therefore not supported.

## What platforms are supported?

- Windows (x86-64, x86)
- Linux (x86-64, x86)
- macOS (x86-64 + Apple Silicon)

Note that Linux support is limited by the version of glibc present on the system, which must be
compatible with the one found in Ubuntu 20.04 (Focal Fossa).

## How do I get started?

1. Download the [latest release][rls] from GitHub
2. Extract the files to your project directory
3. Start (or restart) Godot
4. Open your project settings
5. Make sure "Advanced Settings" is enabled
6. Go to "Physics" and then "3D"
7. Change "Physics Engine" to "JoltPhysics3D"
8. Restart Godot

## What's the plan going forward?

In no particular order, here are some of the bigger items:

- Adding substitutes for the joints, allowing for things like springs
- Adding new types of joints, like Jolt's `DistanceConstraint`
- Adding support for double-precision, allowing for large worlds
- Adding support for iOS and Android
- Making the physics server thread-safe

See the [`v1.0.0`][prj] project board for a more up-to-date overview.

## What settings are there?

See [`docs/settings.md`][set] for information about the project settings available in Godot Jolt.

## How do I build from source?

See [`docs/building.md`][bld] for information about how to build Godot Jolt from source.

## What do the versions mean?

Godot Jolt adheres to [Semantic Versioning][smv], formatted as `<major>.<minor>.<patch>`. The major
version will be incremented when backwards-incompatible API changes are made, the minor version will
be incremented when backwards-compatible API changes are made and the patch version will be
incremented when changes are made that don't affect the API.

"API", in this case, refers to any user-facing parts of the extension, such as nodes, properties,
methods, parameters or project settings.

Note that major version `0.x.y` carries a special meaning in semantic versioning, where even minor
versions may contain backwards-incompatible changes.

See [`CHANGELOG.md`][chl] for details about what notable changes were included in each version.

## What's the license?

Godot Jolt is distributed under the MIT license. See [`LICENSE.txt`][lic] for more details and
[`THIRDPARTY.txt`][trd] for third-party licenses.

[log]: docs/logo.svg
[god]: https://godotengine.org/
[jlt]: https://github.com/jrouwe/JoltPhysics
[jdc]: https://jrouwe.github.io/JoltPhysics/
[rls]: https://github.com/godot-jolt/godot-jolt/releases/latest
[prj]: https://github.com/orgs/godot-jolt/projects/1
[set]: docs/settings.md
[bld]: docs/building.md
[smv]: https://semver.org/spec/v2.0.0.html
[chl]: CHANGELOG.md
[lic]: LICENSE.txt
[trd]: THIRDPARTY.txt
