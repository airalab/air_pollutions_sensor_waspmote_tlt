{ stdenv
, python3Packages
, mkRosPackage
, robonomics_comm 
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "air_pollutions_sensor_waspmote_tlt";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [
    python3Packages.ed25519
    python3Packages.sqlalchemy
    robonomics_comm
  ];

  meta = with stdenv.lib; {
    description = "air_pollutions_sensor_waspmote_tlt package";
    homepage = http://github.com/airalab/air_pollutions_sensor_waspmote_tlt;
    license = licenses.bsd3;
    maintainers = [ "Alisher Khassanov <alisher@aira.life>" ];
  };
}
