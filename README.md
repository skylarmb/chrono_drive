# chrono_drive

Physics-first driving simulator built on [Project Chrono](https://projectchrono.org).

## Build

```sh
git submodule update --init      # chrono/ is pinned to a known-good commit
nix develop                      # toolchain + VSG libraries (see flake.nix)

# 1. Chrono libraries (once, or after bumping the submodule)
cmake -S chrono -B build-chrono -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCH_ENABLE_MODULE_VEHICLE=ON -DCH_ENABLE_MODULE_VSG=ON -DBUILD_DEMOS=OFF
ninja -C build-chrono

# 2. chrono_drive
cmake -B build -G Ninja && ninja -C build
./build/chrono_drive
```

Controls are listed at the top of `src/main.cpp`. Maps are grayscale heightmaps in `data/maps/`.
