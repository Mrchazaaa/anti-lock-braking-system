# Anti-lock Braking System

[![Build](https://github.com/Mrchazaaa/ABS/actions/workflows/build.yml/badge.svg?branch=master)](https://github.com/Mrchazaaa/ABS/actions/workflows/build.yml)
[![Tests](https://github.com/Mrchazaaa/ABS/actions/workflows/test.yml/badge.svg?branch=master)](https://github.com/Mrchazaaa/ABS/actions/workflows/test.yml)

Standalone ABS controller library I developed as part of my Computer Science BSc dissertation. The system aims to accurately model the commercial systems in wide use today by the general public, a difficult task owing to the highly sensitive nature of the source code belonging to automotive manufacturers. This was achieved by replicating model behavior as outlined in "Brakes, Brake control and driver Assistance Systems: Function, Regulation and Components" by K. Reif, where a typical ABS control loop is defined in its various stages (p. 82)..

[View a copy of the dissertation here!](https://charliehowlett.co.uk/ABSConstruction.pdf)

It exposes a small C API for:

- creating and resetting controller state
- stepping the controller from wheel-speed inputs
- retrieving brake-command outputs and debug telemetry

## Install

Clone the repository with its submodules:

```bash
git clone --recurse-submodules https://github.com/Mrchazaaa/ABS.git
cd ABS
```

If you already cloned the repository without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

## Build

Configure and build with CMake:

```bash
cmake -S . -B build
cmake --build build
```

This produces the static library in `build/`.

## Test

Configure and build the test target with CMake, then run the suite with CTest:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
