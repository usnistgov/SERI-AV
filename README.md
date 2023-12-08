# SERI-AV

This repository contains a containerized version of the SERI Autonomous Vehicle demo showcased at the 2023 NIST Autonomous Vehicle Workshop.

## Instructions

1. Build the docker image: `./build_av_image.sh`
2. Start the container: `./run_container.sh`
3. Run the install script inside the container: `./install.sh`
4. Inside the container re-source your bash profile: `source ~/.bashrc`
5. Use the `seri_demo` command to activate demo related macros.
6. Run the demo component macro (e.g. `1-carla_start`)
7. Repeat 5 and 6 for each demo macro (in a new window)

## Notes
Currently, `carla-town-4.zip`, an autoware map used in this demo, is not included in the repository.
Any part of the install script or demo that refers to this file will not run properly. This file will need to be acquired and added to your docker container manually.
