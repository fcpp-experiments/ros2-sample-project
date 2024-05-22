// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file common_data.hpp
 * @brief data common to AP and FW.
 */

#ifndef NODES_COMMON_DATA_H_
#define NODES_COMMON_DATA_H_

#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include "feedback_parser.h"

enum class ProcessingStatus
{
    IDLE, // not used at the moment
    TERMINATING,
    SELECTED
};

//! @brief String representation of a shape.
std::string to_string(ProcessingStatus s);

//! @brief Printing ProcessingStatus.
template <typename O>
O& operator<<(O& o, ProcessingStatus s) {
    o << to_string(s);
    return o;
}

enum class node_type
{
    KIOSK,
    ROBOT
};

//! @brief String representation of a node_type.
std::string to_string(node_type nt);

//! @brief Printing node_type.
template <typename O>
O& operator<<(O& o, node_type s) {
    o << to_string(s);
    return o;
}

struct RobotStatus {       
    float pos_x;
    float pos_y;
    float orient_w;
    float battery_percent_charge; //TODO: add other battery fields;
    feedback::GoalStatus goal_status;
    string goal_code;
    int goal_current_step;
    feedback::DockStatus dock_status;
    feedback::SystemStatus system_status;
};

struct InputGoal {       
    std::string action;        
    std::string goal_code;  
    float pos_start_x;
    float pos_start_y;
    float orient_start_w;
    float pos_end_x;
    float pos_end_y;
    float orient_end_w;
    string source;
    int priority;
    string subcode;
    int step;

        //! @brief Equality operator.
    bool operator==(InputGoal const& m) const {
        return goal_code == m.goal_code;
    }
};

namespace std {
    template <>
    struct hash<InputGoal> {
        //! @brief Produces an hash for a message, combining to and from into a size_t.
        size_t operator()(InputGoal const& m) const {
            return std::hash<std::string>{}(m.goal_code);
        }
    };
}

extern std::unordered_map<std::string,std::vector<RobotStatus>> RobotStatesMap;
extern std::mutex RobotStatesMutex;
extern std::vector<InputGoal> InputGoalList;
extern std::mutex GoalMutex;
#endif
