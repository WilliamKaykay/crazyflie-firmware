/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * platform_bolt.c - platform functionality for the Crazyflie Bolt
 */


#define DEBUG_MODULE "PLATFORM"

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"
#include "socketlink.h"
#include "debug.h"
#include "cfassert.h"

/* Store the UDP port to communicate with CF */
uint16_t crtp_port;
/* Store the cf instance unique Identifiant */
uint8_t cf_id;
/* Store an IP address for the server (CF) */
const char* remote_address;
/* Gazebo world */
const char* world_name;
/* Gazebo model instance name*/
const char* model_name;
/* Gazebo model type name*/
const char* model_sim;
/* Initial pose */
const char* init_pose;

#define DEFAULT_CRTP_PORT 19850

static platformConfig_t configs[] = {
#ifdef CONFIG_SENSORS_SITL
  {
    .deviceType = "SITL",
    .deviceTypeName = "Software-in-the-Loop Simulator",
    .sensorImplementation = SensorImplementation_sitl,
    .physicalLayoutAntennasAreClose = false,
  }
#endif
};

const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs) {
  *nrOfConfigs = sizeof(configs) / sizeof(platformConfig_t);
  return configs;
}

void platformInitHardware() {

}

void platformGetDeviceTypeString(char* deviceTypeString) {
  strcpy(deviceTypeString, "0;SITL");
}

// Config functions ------------------------

const char* platformConfigGetPlatformName() {
  return "sitl";
}

void parse_arguments(int argc, char *argv[]) {
    const char* world_name_ = NULL;
    const char* model_sim_ = NULL;
    const char* model_name_ = NULL;
    const char* remote_address_ = NULL;
    const char* crtp_port_ = NULL;
    const char* cf_id_ = NULL;
    const char* init_pose_ = NULL;

    static struct option long_options[] = {
        {"world", required_argument, 0, 'w'},
        {"model_name", required_argument, 0, 'n'},
        {"model_type", required_argument, 0, 'm'},
        {"crtp_address", required_argument, 0, 'a'},
        {"crtp_port", required_argument, 0, 'p'},
        {"cf_id", required_argument, 0, 'i'},
        {"pose", required_argument, 0, 'o'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int c;

    while ((c = getopt_long(argc, argv, "w:n:m:a:p:i:o:", long_options, &option_index)) != -1) {
        switch (c) {
            case 'w':
                world_name_ = optarg;
                break;
            case 'n':
                model_name_ = optarg;
                break;
            case 'm':
                model_sim_ = optarg;
                break;
            case 'a':
                remote_address_ = optarg;
                break;
            case 'p':
                crtp_port_ = optarg;
                break;
            case 'i':
                cf_id_ = optarg;
                break;
            case 'o':
                init_pose_ = optarg;
                break;
            case '?':
                DEBUG_PRINT("Unknown option: %c\n", optopt);
                ASSERT_FAILED();
                break;
        }
    }

    if (!world_name_)
    {
      world_name_ = "crazysim_default";
    }
    if (!model_sim_)
    {
      model_sim_ = "crazyflie";
    }
    if (!cf_id_)
    {
      cf_id_ = "1";
    } else {
      if (atoi(cf_id_) < 1 || atoi(cf_id_) > 255)
      {
        DEBUG_PRINT("Invalid cf_id: %s\n", cf_id_);
        ASSERT_FAILED();
      }
    }
    if (!model_name_)
    {
      char* model_name_str = malloc(strlen(model_sim_) + 1 + strlen(cf_id_) + 1);
      strcpy(model_name_str, model_sim_);
      strcat(model_name_str, "_");
      strcat(model_name_str, cf_id_);
      model_name_ = model_name_str;
    }
    if (!remote_address_)
    {
      remote_address = "127.0.0.1";
    } else {
      remote_address = remote_address_;
    }
    if (!crtp_port_)
    {
      crtp_port = DEFAULT_CRTP_PORT + atoi(cf_id_) - 1;
    } else {
      crtp_port = atoi(crtp_port_);
    }
    if (!init_pose_)
    {
      init_pose_ = "0,0,0,0,0,0";
    }
    world_name = world_name_;
    model_name = model_name_;
    model_sim = model_sim_;
    cf_id = atoi(cf_id_);
    init_pose = init_pose_;
}
