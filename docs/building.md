# Building

This document contains instructions for how to build Godot Jolt from source.

*See [`hacking.md`][hck] for details aimed at developers.*

## Table of Contents

- [Building on Windows](#building-on-windows)
- [Building on Linux](#building-on-linux)
- [Building on macOS](#building-on-macos)

## Building on Windows

Prerequisites:

- Git 2.25 or newer
- CMake 3.22 or newer
- Python 3.8 or newer
- Visual Studio 2022 (with the "Desktop development with C++" workload)
  - You can also use the more lightweight "Build Tools for Visual Studio 2022" instead
- (Optional) Visual Studio's [clang-cl][ccl] component
  - If you wish to compile with LLVM/clang-cl instead of Visual C++

⚠️ These commands will build binaries for 64-bit systems. If you instead wish to build binaries for
32-bit systems then replace `x64` with `x86`.

⚠️ These commands must be run from one of Visual Studio's "[Native Tools Command Prompt][cmd]" in
order for CMake to find the correct toolchain. Make sure you pick the one appropriate for your
target architecture (x64 or x86).

⚠️ Visual Studio's "Developer Command Prompt" and "Developer PowerShell" will not work when
targeting x64 due to them using to an x86 toolchain. Use the "x64 Native Tools Command Prompt"
instead.

Using Microsoft Visual C++:

```sh
# Generate the build directory
cmake --preset windows-msvc-x64

# Build non-editor binaries and install them into the examples project
cmake --build --preset windows-msvc-x64-distribution

# Build editor binaries and install them into the examples project
cmake --build --preset windows-msvc-x64-editor-distribution

# Install non-editor binaries into your project, as `addons/godot-jolt`
cmake --install build/windows-msvc-x64 --config Distribution --prefix C:/Path/To/Project

# Install editor binaries into your project, as `addons/godot-jolt`
cmake --install build/windows-msvc-x64 --config EditorDistribution --prefix C:/Path/To/Project
```

Using LLVM clang-cl:

```sh
# Generate the build directory
cmake --preset windows-clangcl-x64

# Build non-editor binaries and install them into the examples project
cmake --build --preset windows-clangcl-x64-distribution

# Build editor binaries and install them into the examples project
cmake --build --preset windows-clangcl-x64-editor-distribution

# Install non-editor binaries into your project, as `addons/godot-jolt`
cmake --install build/windows-clangcl-x64 --config Distribution --prefix C:/Path/To/Project

# Install editor binaries into your project, as `addons/godot-jolt`
cmake --install build/windows-clangcl-x64 --config EditorDistribution --prefix C:/Path/To/Project
```

## Building on Linux

Prerequisites:

- Git 2.25 or newer
- CMake 3.22 or newer
- Python 3.8 or newer
- GCC 12 or newer
- (Optional) Clang 15.0.0 or newer
  - If you wish to compile with LLVM/Clang instead of GCC

⚠️ These commands will build binaries for 64-bit systems. If you instead wish to build binaries for
32-bit systems then replace `x64` with `x86`.

Using GCC:

```sh
# Generate the build directory
cmake --preset linux-gcc-x64

# Build non-editor binaries and install them into the examples project
cmake --build --preset linux-gcc-x64-distribution

# Build editor binaries and install them into the examples project
cmake --build --preset linux-gcc-x64-editor-distribution

# Install non-editor binaries into your project, as `addons/godot-jolt`
cmake --install build/linux-gcc-x64 --config Distribution --prefix /path/to/project

# Install editor binaries into your project, as `addons/godot-jolt`
cmake --install build/linux-gcc-x64 --config EditorDistribution --prefix /path/to/project
```

Using Clang:

```sh
# Generate the build directory
cmake --preset linux-clang-x64

# Build non-editor binaries and install them into the examples project
cmake --build --preset linux-clang-x64-distribution

# Build editor binaries and install them into the examples project
cmake --build --preset linux-clang-x64-editor-distribution

# Install non-editor binaries into your project, as `addons/godot-jolt`
cmake --install build/linux-clang-x64 --config Distribution --prefix /path/to/project

# Install editor binaries into your project, as `addons/godot-jolt`
cmake --install build/linux-clang-x64 --config EditorDistribution --prefix /path/to/project
```

## Building on macOS

Prerequisites:

- Git 2.25 or newer
- CMake 3.22 or newer
- Python 3.8 or newer
- Xcode 14.3 or equivalent Xcode Command Line Tools

```sh
# Generate the build directory
cmake --preset macos-clang

# Build non-editor binaries and install them into the examples project
cmake --build --preset macos-clang-distribution

# Build editor binaries and install them into the examples project
cmake --build --preset macos-clang-editor-distribution

# Install non-editor binaries into your project, as `addons/godot-jolt`
cmake --install build/macos-clang --config Distribution --prefix /path/to/project

# Install editor binaries into your project, as `addons/godot-jolt`
cmake --install build/macos-clang --config EditorDistribution --prefix /path/to/project
```

[hck]: hacking.md
[ccl]: https://learn.microsoft.com/en-us/cpp/build/clang-support-msbuild
[cmd]: https://learn.microsoft.com/en-us/cpp/build/building-on-the-command-line
