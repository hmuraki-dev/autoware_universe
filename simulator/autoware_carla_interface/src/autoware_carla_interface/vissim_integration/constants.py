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

# Simulation period/resolution management (see PROTO_VERSION 2 in rpc_protocol.py). The Linux
# side owns the co-simulation period and stops the whole co-simulation once it has elapsed; the
# period written into the .inpx network file is that value plus this margin, so that Vissim itself
# never reaches the end of its simulation period first (Vissim only advances when ticked, so a
# generous margin costs nothing - it only absorbs the few ticks by which Vissim may run ahead of
# the client's own count after a timed-out-but-processed tick request).
VISSIM_SIM_PERIOD_MARGIN_S = 10

# Valid ranges of the Vissim 'Simulation' object attributes written into the .inpx network file
# (from attribute.xlsx shipped with PTV Vissim 2025, confirmed to work with Vissim 2026).
# SimRes is 1-20 time steps per simulation second without the 'Automotive' add-on license (1-1000
# with it); the license-independent range is used here. SimPeriod is in simulation seconds.
VISSIM_MIN_SIM_RES = 1
VISSIM_MAX_SIM_RES = 20
VISSIM_MIN_SIM_PERIOD_S = 1
VISSIM_MAX_SIM_PERIOD_S = 2678400
