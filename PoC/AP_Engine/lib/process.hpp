// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file process.hpp
 * @brief macro utils for process management.
 */

#include "lib/common_data.hpp"

#ifndef NODES_PROCESS_H_
#define NODES_PROCESS_H_

using namespace fcpp;
using namespace fcpp::option::data;
using namespace fcpp::coordination::tags;
using namespace fcpp::coordination;

//! @brief A representation of result of spawn process
using spawn_result_type = std::unordered_map<goal_tuple_type, times_t, fcpp::common::hash<goal_tuple_type>>;

// MACRO
#define PROCESS(NewGoalsList, code) \
    spawn_result_type r =  coordination::spawn(CALL, [&](goal_tuple_type const& g){ \
        status s = status::internal_output; \
        code \
        termination_logic(CALL, s, g); \
        return make_tuple(node.current_time(), s); \
    }, NewGoalsList); \
    manage_termination(CALL, nt, r); \

    //! @brief Termination logic using share (see SHARE termination in ACSOS22 paper)
    FUN void termination_logic(ARGS, status& s, goal_tuple_type const&) {
        bool terminatingMyself = s == status::terminated_output;
        bool terminatingNeigh = nbr(CALL, terminatingMyself, [&](field<bool> nt){
            return any_hood(CALL, nt) or terminatingMyself;
        });
        bool exiting = all_hood(CALL, nbr(CALL, terminatingNeigh), terminatingNeigh);
        if (exiting) s = status::border; 
        else if (terminatingMyself) s = status::internal_output;
    }

    //! @brief Manage termination of the spawn processes.
    FUN void manage_termination(ARGS, node_type nt, spawn_result_type& r) {
        // if process was terminating and now it's terminated, we have to change state machine to IDLE 
        if (node.storage(node_process_status{}) == ProcessingStatus::TERMINATING) {
            // if process has been terminated, it isn't in the result map of spawn
            bool process_found = false;
            for (auto const& x : r)
            {
                auto g = x.first;
                if (common::get<goal_code>(g) == node.storage(node_process_goal{})){
                    process_found = true;
                } 
            }
            if (!process_found) {
                // std::cout << "Process with code " << node.storage(node_process_goal{}) << " not found, so move robot to IDLE" << endl;
                node.storage(node_process_status{}) = ProcessingStatus::IDLE;
            }
        }
    }
    
#endif
