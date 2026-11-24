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

## Patching Chrono

Extend Chrono from `src/` (for example, by subclassing) when you can. Patch Chrono only when the code you must change is not accessible.

`chrono/` tracks the `chrono_drive` branch of [skylarmb/chrono](https://github.com/skylarmb/chrono). That branch holds our patches on top of upstream. The fork's `main` stays an unmodified mirror of upstream.

```sh
# Make a patch: commit in the submodule, push it, then record the new pointer.
git -C chrono commit -am "..." && git -C chrono push
git add chrono && git commit -m "Bump chrono: ..."

git -C chrono log --oneline upstream/main..   # list our patch set

# Update Chrono: rebase our patches onto a newer upstream commit.
git -C chrono fetch upstream && git -C chrono rebase <upstream-commit>
git -C chrono push --force-with-lease
```

After a fresh clone, run `git -C chrono remote add upstream https://github.com/projectchrono/chrono.git`.

## License

BSD 3-Clause (see `LICENSE`). [Project Chrono](https://github.com/projectchrono/chrono) is also BSD-licensed. `data/maps/terrain3.bmp` and the HMMWV specs in `data/vehicle/hmmwv/` (with local changes to the suspension) come from Chrono's data directory and are distributed under Chrono's license, copyright (c) 2016, Project Chrono Development Team.
