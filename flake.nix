{
  description = "chrono_drive: physics-first driving simulator built on Project Chrono";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
      vsg = pkgs.vulkanscenegraph;
      # Libraries every VSG consumer needs (vsgConfig.cmake pkg-config-checks xcb).
      vsgDeps = with pkgs; [ vsg vulkan-headers vulkan-loader glslang libxcb libxau ];

      # vsgXchange: file loaders (OBJ meshes, images, fonts) used by Chrono::VSG.
      # Version pinned to match chrono/contrib/build-scripts/linux/buildVSG.sh.
      vsgXchange = pkgs.stdenv.mkDerivation rec {
        pname = "vsgxchange";
        version = "1.1.12";
        src = pkgs.fetchFromGitHub {
          owner = "vsg-dev";
          repo = "vsgXchange";
          tag = "v${version}";
          hash = "sha256-X1ylp/2vKUp+sGE6ypDalc2dMbWx+N0pXwAoAAG7sIE=";
        };
        nativeBuildInputs = with pkgs; [ cmake pkg-config ];
        buildInputs = vsgDeps ++ [ pkgs.assimp pkgs.freetype ];
      };

      # vsgImGui: Dear ImGui integration (Chrono's on-screen GUI). Bundles imgui/implot as submodules.
      vsgImGui = pkgs.stdenv.mkDerivation rec {
        pname = "vsgimgui";
        version = "0.7.0";
        src = pkgs.fetchFromGitHub {
          owner = "vsg-dev";
          repo = "vsgImGui";
          tag = "v${version}";
          fetchSubmodules = true;
          hash = "sha256-GRHTpMTqK9UgvINZG2oN9zTh0lnBtNw0JdwB4ZT47gk=";
        };
        nativeBuildInputs = with pkgs; [ cmake pkg-config ];
        buildInputs = vsgDeps;
      };
    in
    {
      packages.${system} = { inherit vsgXchange vsgImGui; };

      devShells.${system} = rec {
        default = pkgs.mkShell {
          # Build tools.
          nativeBuildInputs = with pkgs; [
            cmake
            ninja
            pkg-config
            vulkan-tools # vulkaninfo, for debugging GPU detection
          ];
          # Libraries (buildInputs so pkg-config/cmake hooks pick them up).
          buildInputs = vsgDeps ++ (with pkgs; [
            eigen
            vsgXchange
            vsgImGui
            assimp
            freetype
            libxdmcp
          ]);
        };

        # `nix develop .#cuda`: default shell + CUDA toolkit, for GPU modules (CRM/SPH terrain).
        # Uses CUDA's backendStdenv so the host compiler is a GCC version nvcc supports.
        cuda =
          let
            cudaPkgs = (import nixpkgs { inherit system; config.allowUnfree = true; }).cudaPackages_12;
          in
          (pkgs.mkShell.override { stdenv = cudaPkgs.backendStdenv; }) {
            inherit (default) buildInputs;
            nativeBuildInputs = default.nativeBuildInputs ++ [ cudaPkgs.cudatoolkit ];
            # Binaries link against the toolkit's stub libcuda; load the real NixOS driver instead.
            shellHook = ''
              export LD_LIBRARY_PATH=/run/opengl-driver/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
            '';
          };
      };
    };
}
