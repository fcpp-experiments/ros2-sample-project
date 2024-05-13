// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file poc_config.hpp
 * @brief Main configuration of the AP Engine
 */

#ifndef POC_CONFIG_H
#define POC_CONFIG_H

#include <string>
#include <limits>
#include <array>
#include "poc_utils.hpp"

using namespace std;

/* DEFINES */
#define NULL_FLOAT_VALUE        -std::numeric_limits<float>::infinity() //TODO: think about other better empty values
#define NULL_INT_VALUE          -std::numeric_limits<int>::infinity() //TODO: think about other better empty values
#define AP_DEF_COMM_RANGE        20
#define AP_DEF_RETAIN_SEC        2

/* EXTERN */
extern const int ROBOTS_START_INDEX;
extern const std::vector<std::string> ROBOTS; 

/* CONSTANTS */
const string AP_ROBOT_PREFIX_KEY            = "AP_ROBOT_PREFIX";
const string DEFAULT_ROBOT_PREFIX           = "tb3_";
const string AP_ROBOT_COUNT_KEY             = "AP_ROBOT_COUNT";
const int DEFAULT_ROBOT_COUNT               = 5;
const string AP_ROUND_PERIOD_KEY            = "AP_ROUND_PERIOD";
const double DEFAULT_SIDE_X                 = 3.5;
const string AP_SIDE_X_KEY                  = "AP_SIDE_X";
const double DEFAULT_SIDE_Y                 = 6.0;
const string AP_SIDE_Y_KEY                  = "AP_SIDE_Y";
const double DEFAULT_ROUND_PERIOD           = 0.2;
const int DEFAULT_ROBOT_FAIL_TIMEOUT_SEC    = 5;
const string AP_SIMULATOR_OFFSET_X_KEY      = "AP_SIMULATOR_OFFSET_X";
const double DEFAULT_SIMULATOR_OFFSET_X     = 0.4;
const string AP_SIMULATOR_OFFSET_Y_KEY      = "AP_SIMULATOR_OFFSET_Y";
const double DEFAULT_SIMULATOR_OFFSET_Y     = 0.0;
const string AP_CUSTOM_GRAPH_DIAMETER_KEY   = "AP_CUSTOM_GRAPH_DIAMETER";
const int DEFAULT_CUSTOM_GRAPH_DIAMETER     = NULL_INT_VALUE;
const string AP_BG_IMAGE                    = "AP_BG_IMAGE";
const string DEFAULT_BG_IMAGE               = "library.png";
const string ABORT_ACTION                   = "ABORT";
const string GOAL_ACTION                    = "GOAL";
// from ENV
const int ROBOTS_COUNT                      = read_int_env(AP_ROBOT_COUNT_KEY, DEFAULT_ROBOT_COUNT);
const std::string ROBOT_PREFIX              = read_string_env(AP_ROBOT_PREFIX_KEY, DEFAULT_ROBOT_PREFIX);
const double ROUND_PERIOD                   = read_double_env(AP_ROUND_PERIOD_KEY, DEFAULT_ROUND_PERIOD);
const double AXIS_X_LENGTH                  = read_double_env(AP_SIDE_X_KEY, DEFAULT_SIDE_X);
const double AXIS_Y_LENGTH                  = read_double_env(AP_SIDE_Y_KEY, DEFAULT_SIDE_Y);
const double SIMULATOR_OFFSET_X             = read_double_env(AP_SIMULATOR_OFFSET_X_KEY, DEFAULT_SIMULATOR_OFFSET_X);
const double SIMULATOR_OFFSET_Y             = read_double_env(AP_SIMULATOR_OFFSET_Y_KEY, DEFAULT_SIMULATOR_OFFSET_Y);
const int CUSTOM_GRAPH_DIAMETER             = read_int_env(AP_CUSTOM_GRAPH_DIAMETER_KEY, DEFAULT_CUSTOM_GRAPH_DIAMETER);
const std::string BG_IMAGE                  = read_string_env(AP_BG_IMAGE, DEFAULT_BG_IMAGE);

/* DEFINES */
#if   defined(RUN_SIMULATION)
    #ifndef FCPP_SYSTEM
        #define FCPP_SYSTEM                 FCPP_SYSTEM_GENERAL
    #endif
    #ifndef FCPP_ENVIRONMENT
        #define FCPP_ENVIRONMENT            FCPP_ENVIRONMENT_SIMULATED
    #endif
    const std::vector<std::string> ROBOTS   = generate_robot_names(ROBOT_PREFIX, 1, ROBOTS_COUNT);
#endif
#ifndef AP_ENGINE_DEBUG
    #define AP_ENGINE_DEBUG                 false
#endif
#ifndef AP_COMM_RANGE
    #define AP_COMM_RANGE                   AP_DEF_COMM_RANGE
#endif 
#ifndef AP_RETAIN_SEC
    #define AP_RETAIN_SEC                   AP_DEF_RETAIN_SEC
#endif


/* SIMULATOR */
const float NODE_SIZE              = 0.2;
const float NODE_SHADOW_SIZE       = 0.3;
const float LABEL_SIZE             = 0.006;

/* WATCHER */
#define INPUT_FOLDER_BASE_PATH  "../../../Storage/" 
#define OUTPUT_FOLDER_BASE_PATH "../../../Storage/" 
#define COLUMN_DELIMITER        ';'
#define POLLING_MS              10

const string ROBOTS_PLACEHOLDER                         = "#ROBOT";
const string INPUT_FOLDER_FROM_ROBOTS_FEEDBACK          = string("from_robot/") + ROBOTS_PLACEHOLDER + string("/to_ap/feedback");
const string INPUT_FOLDER_FROM_USER_GOALS               = "from_user/goals";

const string OUTPUT_FOLDER_TO_ROBOTS_ACTION             = string("from_ap/to_robot/actions/") + ROBOTS_PLACEHOLDER;

#endif
