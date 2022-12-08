# Building

This document contains instructions for how to build Godot Jolt from source.

*See [`hacking.md`][hck] for details aimed at developers.*

## Table of Contents

- [Building on Windows](#building-on-windows)
- [Building on Linux](#building-on-linux)
- [Building on macOS](#building-on-macos)

## Building on Windows

Prerequisites:

- CMake 3.22 or newer
- Python 3.8 or newer
- Visual Studio 2022 (with the "Desktop development with C++" workload)
- (Optional) Visual Studio's [clang-cl][ccl] component
  - If you wish to compile with LLVM/clang-cl instead of Visual C++.

⚠️ These commands will build binaries for 64-bit systems. If you instead wish to build binaries for
32-bit systems then replace `x64` with `x86`.

⚠️ Either of these need to be run from one of Visual Studio's [Native Tools Command Prompt][cmd] for
CMake to find the correct toolchain. Make sure you pick the one appropriate for your target
architecture (x64 or x86).

Using Microsoft Visual C++:

```pwsh
cmake --preset windows-msvc-x64
cmake --build --preset windows-msvc-x64-distribution
cmake --build --preset windows-msvc-x64-editor-distribution
```

Using LLVM clang-cl:

```pwsh
cmake --preset windows-clangcl-x64
cmake --build --preset windows-clangcl-x64-distribution
cmake --build --preset windows-clangcl-x64-editor-distribution
```

## Building on Linux

Prerequisites:

- CMake 3.22 or newer
- Python 3.8 or newer
- (Optional) Clang 14.0.0 or newer
- (Optional) GCC 11 or newer

⚠️ These commands will build binaries for 64-bit systems. If you instead wish to build binaries for
32-bit systems then replace `x64` with `x86`.

Using Clang:

```pwsh
cmake --preset linux-clang-x64
cmake --build --preset linux-clang-x64-distribution
cmake --build --preset linux-clang-x64-editor-distribution
```

Using GCC:

```pwsh
cmake --preset linux-gcc-x64
cmake --build --preset linux-gcc-x64-distribution
cmake --build --preset linux-gcc-x64-editor-distribution
```

## Building on macOS

Prerequisites:

- CMake 3.22 or newer
- Python 3.8 or newer
- Xcode 14.1 or equivalent Xcode Command Line Tools

```pwsh
cmake --preset macos-clang
cmake --build --preset macos-clang-distribution
cmake --build --preset macos-clang-editor-distribution
```

[hck]: hacking.md
[ccl]: https://learn.microsoft.com/en-us/cpp/build/clang-support-msbuild
[cmd]: https://learn.microsoft.com/en-us/cpp/build/building-on-the-command-line
