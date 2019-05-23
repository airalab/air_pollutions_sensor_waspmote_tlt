all: secrets.h

secrets.h:
	nix-shell -p "haskell.packages.ghc865.ghcWithPackages (pkgs: with pkgs; [ cryptonite ])" --run ./generate.hs > secrets.h
