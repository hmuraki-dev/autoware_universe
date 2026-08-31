#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Vendored without modification from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/vissim_integration/constants.py`). See ../NOTICE.md.
""" This module defines constants used for the vissim-carla co-simulation. """

# ==================================================================================================
# -- constants -------------------------------------------------------------------------------------
# ==================================================================================================

INVALID_ACTOR_ID = -1
CARLA_SPAWN_OFFSET_Z = 25.0  # meters

# Maximum distance of a Vissim veh/ped from a simulator veh/ped to be seen by the simulator (<=0
# means unlimited radius).
VISSIM_VISIBILITY_RADIUS = 0.0

# Maximum number of simulator vehicles/pedestrians/detectors (to be passed to Vissim).
VISSIM_MAX_SIMULATOR_VEH = 5000
VISSIM_MAX_SIMULATOR_PED = 5000
VISSIM_MAX_SIMULATOR_DET = 500

# Maximum number of vissim vehicles/pedestrians/signal groups (to be passed to the simulator).
VISSIM_MAX_VISSIM_VEH = 5000
VISSIM_MAX_VISSIM_PED = 5000
VISSIM_MAX_VISSIM_SIGGRP = 5000

# Vehicle type requested for newly created Driving-Simulator (CARLA) vehicles (used as
# Simulator_Veh_Data.VehicleType when Create == True). 0 makes vissim fall back to the default
# Driving-Simulator vehicle type configured in the .inpx network settings (see
# DrivingSimulatorProxy.h, Simulator_Veh_Data.VehicleType / Create).
VISSIM_DEFAULT_VEHICLE_TYPE = 0

# VISSIM Vehicle data constants.
NAME_MAX_LENGTH = 100
MAX_UDA = 16
