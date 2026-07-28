# Vendored from CARLA's official SUMO co-simulation bridge
# (https://github.com/carla-simulator/carla, Co-Simulation/Sumo/sumo_integration),
# MIT License, Copyright (c) 2020 Computer Vision Center (CVC) at the
# Universitat Autonoma de Barcelona (UAB).
#
# See NOTICE in this directory for details on what was vendored and modified.
#
# NOTE: This package intentionally has no eager imports. `sumo_simulation.py`
# requires `traci`/`sumolib` (only available when SUMO_HOME is configured), so
# submodules must be imported explicitly by callers that need SUMO support.
