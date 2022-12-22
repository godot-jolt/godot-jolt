# ![Godot Jolt][bnr]

Godot Jolt is a native extension for the [Godot game engine][gdt] (4.0 and later) that allows you to
use the [Jolt physics engine][jlt] to power Godot's 3D physics.

⚠️ Godot Jolt is still in its very early stages and is broken/unfinished in a lot of places.

## Compatibility

You can track the progress of the implementation for each of the node types that Godot exposes
through its extension API using the issues listed below.

- Objects
  - [`Area3D`][i95]
  - [`StaticBody3D`][i92]
  - [`RigidBody3D`][i93]
  - [`CharacterBody3D`][i94]
  - [`SoftBody3D`][i114]
- Shapes
  - [`CollisionShape3D`][i96]
  - [`SphereShape3D`][i97]
  - [`BoxShape3D`][i98]
  - [`CapsuleShape3D`][i99]
  - [`CylinderShape3D`][i100]
  - [`ConvexPolygonShape3D`][i101]
  - [`ConcavePolygonShape3D`][i102]
  - [`HeightMapShape3D`][i103]
  - [`WorldBoundaryShape3D`][i104]
  - [`SeparationRayShape3D`][i105]
- Joints
  - [`PinJoint3D`][i106]
  - [`HingeJoint3D`][i107]
  - [`ConeTwistJoint3D`][i108]
  - [`SliderJoint3D`][i109]
  - [`Generic6DOFJoint3D`][i110]
- Miscellaneous
  - [`PhysicsDirectBodyState3D`][i111]
  - [`PhysicsDirectSpaceState3D`][i112]
  - [`PhysicsServer3D`][i113]

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
[i92]: https://github.com/godot-jolt/godot-jolt/issues/92
[i93]: https://github.com/godot-jolt/godot-jolt/issues/93
[i94]: https://github.com/godot-jolt/godot-jolt/issues/94
[i95]: https://github.com/godot-jolt/godot-jolt/issues/95
[i96]: https://github.com/godot-jolt/godot-jolt/issues/96
[i97]: https://github.com/godot-jolt/godot-jolt/issues/97
[i98]: https://github.com/godot-jolt/godot-jolt/issues/98
[i99]: https://github.com/godot-jolt/godot-jolt/issues/99
[i100]: https://github.com/godot-jolt/godot-jolt/issues/100
[i101]: https://github.com/godot-jolt/godot-jolt/issues/101
[i102]: https://github.com/godot-jolt/godot-jolt/issues/102
[i103]: https://github.com/godot-jolt/godot-jolt/issues/103
[i104]: https://github.com/godot-jolt/godot-jolt/issues/104
[i105]: https://github.com/godot-jolt/godot-jolt/issues/105
[i106]: https://github.com/godot-jolt/godot-jolt/issues/106
[i107]: https://github.com/godot-jolt/godot-jolt/issues/107
[i108]: https://github.com/godot-jolt/godot-jolt/issues/108
[i109]: https://github.com/godot-jolt/godot-jolt/issues/109
[i110]: https://github.com/godot-jolt/godot-jolt/issues/110
[i111]: https://github.com/godot-jolt/godot-jolt/issues/111
[i112]: https://github.com/godot-jolt/godot-jolt/issues/112
[i113]: https://github.com/godot-jolt/godot-jolt/issues/113
[i114]: https://github.com/godot-jolt/godot-jolt/issues/114
[bld]: docs/building.md
[lic]: LICENSE.txt
[cpr]: COPYRIGHT.txt
