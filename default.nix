let
  nixpkgs = import <nixpkgs>;
  pkgs = nixpkgs {};
in
  pkgs.stdenv.mkDerivation rec {
    name = "pololu-usb-to-i2c-adapter-firmware";
    src = builtins.filterSource
      (path: type: (builtins.substring 0 5 (baseNameOf path)) != "build")
      ./firmware;
    buildInputs = [ pkgs.gcc-arm-embedded pkgs.cmake pkgs.ninja ];
    builder = ./firmware/nix_builder.sh;
  }
