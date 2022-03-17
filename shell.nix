let
    pkgs = import <nixpkgs> {};
in let
    osi3 = pkgs.python39.pkgs.buildPythonPackage rec {
        pname = "osi3";
        version = "3.4.0";
        src = fetchGit {
            url = "https://github.com/OpenSimulationInterface/open-simulation-interface.git";
            ref = "v3.4.0";
        };
        nativeBuildInputs = [ pkgs.protobuf ];
        buildInputs = [ pkgs.python39.pkgs.pyyaml ];
        propagatedBuildInputs = [ pkgs.python39.pkgs.protobuf ];
    };
in let
    python = pkgs.python39.withPackages (ps: [ osi3 ]);
in python.env
