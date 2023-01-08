# Hacking

*See [`building.md`][bld] for per-platform prerequisites.*

This document contains details and guidelines aimed at developers looking to hack on Godot Jolt.

Most CMake [generators][gen] should be compatible, meaning you can generate project files for
environments like Visual Studio, Xcode or straight Makefiles. This is done either through their CLI
or GUI application. Alternatively you can use the [presets](#presets) described below.

## Table of Contents

- [Options](#options)
- [Presets](#presets)
- [User Presets](#user-presets)
- [Formatting](#formatting)
- [Linting](#linting)

## Options

These are the project-specific CMake options that are available. They are only relevant if you
decide *not* to use the presets described [below](#presets), but you can also override the presets'
defaults by passing `-DGDJOLT_SOME_VARIABLE=VALUE` to CMake.

- `GDJOLT_X86_INSTRUCTION_SET`
  - Sets the minimum required CPU instruction set when compiling for x86.
  - ⚠️ This flag is not available on Apple platforms, due to universal binaries not supporting it.
  - Possible values are [`NONE`, `SSE2`, `AVX`, `AVX2`, `AVX512`]
  - Default is `SSE2`
    - This is to conform with Godot, which doesn't utilize anything newer than SSE2.
    - There will be separate releases on GitHub for each of the instruction sets.
      - Alternatively all will be bundled and loaded based on `CPUID`.
- `GDJOLT_CROSS_PLATFORM_DETERMINISTIC`
  - Compiles in such a way as to attempt to keep things deterministic across *different* platforms.
  - ⚠️ While this flag makes Jolt itself deterministic, the Godot engine and/or this extension may
    not be, and likely aren't, which makes this flag nothing but a performance cost.
  - You do not need this for determinism on the same platform (assuming identical binary).
  - Default is `FALSE`.
- `GDJOLT_INTERPROCEDURAL_OPTIMIZATION`
  - Enables interprocedural optimizations for any optimized builds, also known as link-time
    optimizations or link-time code generation.
  - Default is `TRUE`.
- `GDJOLT_PRECOMPILE_HEADERS`
  - Enables precompiling of header files that don't change often, like external ones, which speeds
    up compilations.
  - Disabling this will make it so any precompiled headers gets force-included instead.
  - Default is `TRUE`.
- `GDJOLT_STATIC_RUNTIME_LIBRARY`
  - Whether to statically link against the platform-specific C++ runtime, for added portability.
  - ⚠️ This flag is not available on Apple platforms.
  - Default is `TRUE`.

## Presets

There are configuration and build presets available that utilize the relatively new
[`CMakePresets.json`][prs]. These make for a less verbose command-line interface, but also help
unify behavior across environments. Visual Studio (through a [component][mvs]) and Visual Studio
Code (through an [extension][vsc]) both support these and lets you choose these presets from within
the editor.

All these presets default to using the `Ninja Multi-Config` generator which uses the [Ninja][nnj]
build system. The binaries for Ninja are bundled in this repository (under `tools/ninja`) and do not
need to installed separately.

The following configuration presets are currently available:

- `windows-msvc-x64` (Microsoft Visual C++, x86-64)
- `windows-msvc-x86` (Microsoft Visual C++, x86)
- `windows-clangcl-x64` (LLVM clang-cl, x86-64)
- `windows-clangcl-x86` (LLVM clang-cl, x86)
- `linux-gcc-x64` (GCC, x86-64)
- `linux-gcc-x86` (GCC, x86)
- `linux-clang-x64` (LLVM Clang, x86-64)
- `linux-clang-x86` (LLVM Clang, x86)
- `macos-clang` (Apple Clang, [universal][uvb])

One of the following suffixes are then applied to the configuration presets to create the build
preset:

- `-debug`
- `-development`
- `-distribution`
- `-editor-debug`
- `-editor-development`
- `-editor-distribution`

`-debug` signifies a build with optimizations disabled and extension-related debugging features
enabled.

`-development` signifies a build with optimizations enabled and extension-related debugging features
enabled.

`-distribution` signifies a build with optimizations enabled and extension-related debugging
features disabled.

`-editor` signifies what Godot calls a "debug" build and will build the version of the library that
gets used when inside the Godot editor as well as what gets bundled when exporting a "debug" build
from Godot.

You then use these presets like so:

```pwsh
# Using the above mentioned configure preset
cmake --preset windows-msvc-x64

# Using the above mentioned build preset
cmake --build --preset windows-msvc-x64-editor-debug
```

Note that **all** configurations currently include debug symbols when building from source. Debug
symbols will be stripped from `-distribution` binaries and provided as a separate download on
GitHub.

## User Presets

CMake offers the ability to have local/personal presets through a `CMakeUserPresets.json` file. This
file lets you define your own presets that can inherit from and extend the presets found in
`CMakePresets.json`. These user presets will show up in editors that support it just like the
regular presets.

It is encouraged to have your user presets inherit from the `dev-base` preset, which enables certain
developer-specific settings, like more pedantic CMake warnings.

Simply create a `CMakeUserPresets.json` next to `CMakePresets.json` and have it look something like
this:

```json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "dev",
      "inherits": ["windows-msvc-x64", "dev-base"],
      "displayName": "MSVC, 64-bit, Development"
    }
  ],
  "buildPresets": [
    {
      "name": "dev-debug",
      "inherits": ["install-base"],
      "configurePreset": "dev",
      "configuration": "EditorDebug",
      "displayName": "EditorDebug"
    },
    {
      "name": "dev-development",
      "inherits": ["install-base"],
      "configurePreset": "dev",
      "configuration": "EditorDevelopment",
      "displayName": "EditorDevelopment"
    },
    {
      "name": "dev-distribution",
      "inherits": ["install-base"],
      "configurePreset": "dev",
      "configuration": "EditorDistribution",
      "displayName": "EditorDistribution"
    }
  ]
}
```

Then you can configure/build it like any other preset:

```pwsh
cmake --preset dev
cmake --build --preset dev-debug
```

⚠️ If your build presets don't inherit from `install-base`, either directly or indirectly, you will
have to explicitly pass `--target install` to the build command-line for your output binaries to end
up in the `bin` directory, which they need to be for the examples project to be able to load them.

## Formatting

Prerequisites:

- PowerShell 7.2.7 or newer

[clang-format][clf] is used to format code in Godot Jolt. There are numerous extensions that allow
you to use clang-format from within the editor of your choosing.

⚠️ The clang-format configuration that Godot Jolt uses is written for **LLVM 14.0** and won't work
with earlier versions, possibly not newer ones either.

There is a PowerShell script, `scripts/run_clang_format.ps1`, that runs clang-format on all source
files in the directory you provide, with the option to fix any errors it encounters.

To see if you have any formatting errors:

```pwsh
./scripts/run_clang_format.ps1 -SourcePath ./src
```

To also automatically fix any formatting errors it might encounter:

```pwsh
./scripts/run_clang_format.ps1 -SourcePath ./src -Fix
```

## Linting

Prerequisites:

- PowerShell 7.2.7 or newer

[clang-tidy][clt] is used to lint code in Godot Jolt. There are numerous extensions that allow you
to use clang-tidy from within the editor of your choosing.

⚠️ The clang-tidy configuration that Godot Jolt uses is written for **LLVM 14.0** and won't work
with earlier versions, possibly not newer ones either.

Clang-tidy compiles the code in order to analyze it. To achieve this it needs to know what compiler
front-end to use, as well as what compiler flags to compile each file with. This is provided through
a [compile_commands.json][cdb] file. This file is generated by CMake when you generate your build
files, so you need to provide clang-tidy with the path to your generated build files as shown below.

There is a PowerShell script, `scripts/run_clang_tidy.ps1`, that runs clang-tidy on all source files
in the directory you provide, with the option to try to fix any errors it encounters.

⚠️ Because clang-tidy actually compiles the code of **every** configuration (for your chosen
platform), it also (sadly) needs the precompiled header file of **every** configuration. Therefore
you need to make sure to build **every** configuration before you run clang-tidy, otherwise you will
get a lot of weird errors.

⚠️ Because clang-tidy compiles MSVC configurations with what's effectively `clang-cl`, it seems to
struggle to use the precompiled headers that the `windows-msvc` configurations produce. Therefore it
is recommended to compile the `windows-clangcl` configurations instead and point clang-tidy to
those.

To build all configurations:

```pwsh
cmake --build --preset windows-clangcl-x64-debug
cmake --build --preset windows-clangcl-x64-development
cmake --build --preset windows-clangcl-x64-distribution
cmake --build --preset windows-clangcl-x64-editor-debug
cmake --build --preset windows-clangcl-x64-editor-development
cmake --build --preset windows-clangcl-x64-editor-distribution
```

To see if you have any linting errors:

```pwsh
./scripts/run_clang_tidy.ps1 -SourcePath ./src -BuildPath ./build/windows-clangcl-x64
```

To also automatically fix any linting errors it might encounter:

⚠️ This is very slow, as `-Fix` can't run parallel instances of clang-tidy.

```pwsh
./scripts/run_clang_tidy.ps1 -SourcePath ./src -BuildPath ./build/windows-clangcl-x64 -Fix
```

[bld]: building.md
[gen]: https://cmake.org/cmake/help/v3.22/manual/cmake-generators.7.html
[prs]: https://cmake.org/cmake/help/v3.22/manual/cmake-presets.7.html
[mvs]: https://learn.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio
[vsc]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools
[nnj]: https://ninja-build.org/
[uvb]: https://en.wikipedia.org/wiki/Universal_binary
[clf]: https://releases.llvm.org/14.0.0/tools/clang/docs/ClangFormat.html
[clt]: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/clang-tidy/index.html
[cdb]: https://clang.llvm.org/docs/JSONCompilationDatabase.html
