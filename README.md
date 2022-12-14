# ![Godot Jolt][bnr]

Godot Jolt is a native extension for the [Godot game engine][gdt] (4.0 and later) that allows you to
use the [Jolt physics engine][jlt] to power Godot's 3D physics.

⚠️ Godot Jolt is still in its very early stages and is broken/unfinished in a lot of places.

## Features

Godot Jolt is primarily meant to be a drop-in replacement for Godot Physics, utilizing the same node
types that you would use today, such as `StaticBody3D` or `RigidBody3D`. However, due to mismatching
feature sets between Godot Physics and Jolt there will be certain gaps in functionality, such as a
lack of support for `SoftBody3D` amongst other things.

You can track the progress of this work [here][cpb].

## Platforms

Godot Jolt aims to support most platforms that Godot can be deployed to. Currently only the major
desktop platforms are supported. These include:

- Windows (x86-64, x86)
- Linux (x86-64, x86)
- macOS (x86-64 + Apple Silicon)

## Download

Due to the current state of Godot Jolt there are no precompiled binaries available for download.
Eventually they will be provided here on GitHub and wherever Godot allows distribution of native
extensions.

## Building

See [`docs/building.md`][bld] for details on how to build Godot Jolt from source.

## License

This project is licensed under the MIT license. See [`LICENSE.txt`][lic] for details. See
[`COPYRIGHT.txt`][cpr] for third-party licenses and copyright notices.

[bnr]: docs/banner.png
[gdt]: https://godotengine.org/
[jlt]: https://github.com/jrouwe/JoltPhysics
[cpb]: https://github.com/godot-jolt/godot-jolt/issues/117
[bld]: docs/building.md
[lic]: LICENSE.txt
[cpr]: COPYRIGHT.txt
