# ![Godot Jolt][bnr]

Godot Jolt is a native extension for the [Godot game engine][gdt] that allows you to use the [Jolt
physics engine][jlt] to power Godot's 3D physics.

⚠️ Godot Jolt is still in its very early stages and is broken/unfinished in a lot of places.

## Features

To begin with the focus will be on implementing the existing physics API that Godot exposes through
GDExtension. During this time the feature set will be lesser to that of Godot Physics, due to Jolt
not supporting things like soft-body physics or double-precision. Eventually any new features that
Jolt provides will be exposed as custom node types in the editor.

## Platforms

Godot Jolt aims to support most platforms that Godot can be deployed to. Currently only the major
desktop platforms are supported. These include:

- Windows (x86-64)
- Linux (x86-64)
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
[bld]: docs/building.md
[lic]: LICENSE.txt
[cpr]: COPYRIGHT.txt
