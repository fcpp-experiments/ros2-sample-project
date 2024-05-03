// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file ap_engine.hpp
 * @brief AP code for the engine.
 * 
 * It's used for simulation and embedded deployment. 
 */

#ifndef NODES_AP_ENGINE_H_
#define NODES_AP_ENGINE_H_

#include <string>
#include <ctime>
#include <cmath>

#include "lib/poc_config.hpp"
#include "lib/fcpp.hpp"

#include "lib/common_data.hpp"
#include "lib/ap_engine_setup.hpp"
#include "lib/action_writer.hpp"
#include "lib/process.hpp"


// DEFINES
#define ANY_GOALS                "ANY"

namespace fcpp
{

    namespace coordination
    {    

        using namespace fcpp::coordination::tags;
        using namespace fcpp::option::data;
        using spawn_result_type = std::unordered_map<goal_tuple_type, times_t, fcpp::common::hash<goal_tuple_type>>;

        // UTILS AP

        //! @brief Change color of node, passing in argument
        FUN void change_color(ARGS, fcpp::color color) { CODE
            node.storage(left_color{})    = fcpp::color(color);
            node.storage(right_color{})   = fcpp::color(color);
            node.storage(node_color{})    = fcpp::color(color);
        }

        //! @brief Blink color of node
        FUN void blink_computing_color(ARGS) { CODE
            // TODO: implement me
        }

        //! @brief Get robot name from AP node_uid
        FUN string get_real_robot_name(ARGS, device_t node_uid) { CODE
            return get_robot_name(ROBOTS, static_cast<int>(node_uid));
        }

        //! @brief Update in the storage the tag "node_ext_goal_update_info"
        FUN void update_last_goal_update_time(ARGS, std::string goal_code, feedback::GoalStatus goal_status) { CODE
            std::time_t now = std::time(nullptr);
            std::asctime(std::localtime(&now));
            fcpp::option::data::external_status_tuple_type ext_status_tuple = common::make_tagged_tuple<node_ext_goal_update_time, node_ext_goal_status>(
                    now,
                    goal_status
                );
            node.storage(node_ext_goal_update_info{})[goal_code] = ext_status_tuple;
            node.storage(node_ext_goal_update_info{})[string(ANY_GOALS)] = ext_status_tuple;
        }

        //! @brief Add current goal to computing map
        FUN void add_goal_to_computing_map(ARGS, goal_tuple_type const& g) { CODE
            node.storage(node_process_computing_goals{})[common::get<goal_code>(g)] = g;
        }

        //! @brief Remove current goal to computing map
        FUN void remove_goal_from_computing_map(ARGS, std::string goal_code) { CODE
            node.storage(node_process_computing_goals{}).erase(goal_code);
        }

        //! @brief Check if exists other prioritised goals that are different from current
        FUN bool check_if_other_prioritised_goal_exists(ARGS, std::string current_goal_code) { CODE
            auto map = node.storage(node_process_computing_goals{});
            auto found = std::find_if(map.begin(), map.end(), [&](const auto& p){
                return 
                    common::get<goal_code>(p.second) != current_goal_code &&
                    common::get<goal_priority>(p.second) > 0;
            });
            return found != map.end();
        }

        //! @brief Compute new rank for node, using battery and distance from goal
        FUN real_t run_rank_node(ARGS, float percent_charge, node_type nt, goal_tuple_type const& g) { CODE
            // compute distance to target
            real_t dist = norm(node.position() - make_vec(common::get<goal_pos_end_x>(g), common::get<goal_pos_end_y>(g), 0));
            real_t rank = (nt == node_type::ROBOT)
                                ? dist * (1.0 - percent_charge )
                                : INF;
            // rank setted to INF if:
            auto is_current_goal =  [&]()->bool {return node.storage(node_process_goal{}) == common::get<goal_code>(g);};
            auto is_goal_failed =   [&]()->bool {return node.storage(node_ext_goal_status{}) == feedback::GoalStatus::FAILED;};
            auto is_goal_running =  [&]()->bool {return node.storage(node_ext_goal_status{}) == feedback::GoalStatus::RUNNING;};
            auto is_docking =   [&]()->bool {return node.storage(node_ext_docking_status{}) == feedback::DockStatus::RUNNING;};
            auto is_process_selected = [&]()->bool {return node.storage(node_process_status{}) == ProcessingStatus::SELECTED;};
            auto is_goal_priority_0 = [&]()->bool {return common::get<goal_priority>(g) == 0;};
            auto exists_prioritised_goal = [&]()->bool {return check_if_other_prioritised_goal_exists(CALL, common::get<goal_code>(g));};
            if (
                    // node is failed for current goal and not already selected for something
                    (is_goal_failed() && is_current_goal() && !is_process_selected()) ||
                    // or it's already selected or running for another goal
                    ((is_goal_running() || is_process_selected()) && !is_current_goal()) ||
                    // or if it's processing not prioritised goal, but there are other some prioritised goals
                    (is_goal_priority_0() && exists_prioritised_goal()) ||
                    // or it's returning to its initial position
                    is_docking()
                ) {
                rank = INF;
            }
            // rank setted to INF if percent charge of battery is 0.0
            else if (percent_charge == 0.0) {
               rank = INF;
            }
            // rank set to dist * 0.01 when battery is fully charged, otherwise the rank will be always 0:
            // the case when percent_charge is 1.0 (100%) becomes like percent_charge equals to 0.99 (99%)
            else if (percent_charge >= 1.0) {
                rank = dist * 0.01;
            }

            if (AP_ENGINE_DEBUG) {
                std::cout << "Computed local rank from node " << node.uid << " and for goal " << common::get<goal_code>(g) << " is: " << rank << endl;   
            }
            return rank;
        }

        //! @brief Send "stop" command to robot and change "node_process_status" after it
        FUN void send_stop_command_to_robot(ARGS, string action, device_t node_uid, goal_tuple_type const& g, ProcessingStatus ps) { CODE
            std::string robot_chosen = get_real_robot_name(CALL, node_uid);
            std::cout << "Robot "   << robot_chosen  << " is chosen for " <<
              "receiving ABORT command for goal " << common::get<goal_code>(g) << endl;
            std::cout << endl;

            action::ActionData action_data = {
                .action = action,
                .goal_code = common::get<goal_code>(g),
                .robot = robot_chosen,
                .pos_start_x = common::get<goal_pos_start_x>(g),
                .pos_start_y = common::get<goal_pos_start_y>(g),
                .orient_start_w = common::get<goal_orient_start_w>(g),
                .pos_end_x = common::get<goal_pos_end_x>(g),
                .pos_end_y = common::get<goal_pos_end_y>(g),
                .orient_end_w = common::get<goal_orient_end_w>(g),
                .goal_step  = common::get<goal_step>(g)
            };
            
            action::manager::ActionManager::new_action(action_data);

            // clear goal info if in idle
            if (ProcessingStatus::IDLE == ps) {
                node.storage(node_process_goal{}) = "";
            }

            // clear processing status
            node.storage(node_process_status{}) = ps;
        }

        // STATE MACHINE

        //! @brief Manage state machine when battery is discharged
        FUN void manage_battery_discharged_node(ARGS, RobotStatus rs) { CODE
            change_color(CALL, fcpp::color(fcpp::coordination::discharged_color));
        }

        //! @brief Manage state machine when robot is running a goal
        FUN void manage_running_goal_status(ARGS, RobotStatus rs) { CODE
            change_color(CALL, fcpp::color(fcpp::coordination::running_color));
        }

        //! @brief Manage state machine when robot is running a docking action
        FUN void manage_running_dock_status(ARGS, RobotStatus rs) { CODE
            change_color(CALL, fcpp::color(fcpp::coordination::docking_color));
        }

        //! @brief Manage state machine when robot has reached the goal
        FUN void manage_reached_goal_status(ARGS, RobotStatus rs) { CODE
            fcpp::color new_color;
            // and previous state is NOT REACHED: new color is "reached" and deletes goal storage
            if (feedback::GoalStatus::REACHED != node.storage(node_ext_goal_status{})) {
                new_color = fcpp::color(fcpp::coordination::reached_goal_color);
                // set to terminating processing status
                node.storage(node_process_status{}) = ProcessingStatus::TERMINATING;
                // update time 
                update_last_goal_update_time(CALL, node.storage(node_process_goal{}), feedback::GoalStatus::REACHED);

            // and if process node is SELECTED, then AP has selected new node and simulation is ready to start    
            } else if (node.storage(node_process_status{}) == ProcessingStatus::SELECTED) {
                new_color = fcpp::color(fcpp::coordination::running_color);
            
            // otherwise: new color is "reached"
            } else {
                new_color = fcpp::color(fcpp::coordination::reached_goal_color);
            }
            change_color(CALL, new_color);
        }

        //! @brief Manage state machine when robot has reached the goal
        FUN void manage_illegal_goal_status(ARGS, RobotStatus rs) { CODE
            fcpp::color new_color;
            // and previous state is NOT ILLEGAL: new color is "illegal" and deletes goal storage
            if (feedback::GoalStatus::ILLEGAL != node.storage(node_ext_goal_status{})) {
                new_color = fcpp::color(fcpp::coordination::illegal_color);
                // set to terminating processing status
                node.storage(node_process_status{}) = ProcessingStatus::TERMINATING;
                // update time
                update_last_goal_update_time(CALL, node.storage(node_process_goal{}), feedback::GoalStatus::ILLEGAL);
                std::cout << "Terminating illegal goal with code " << rs.goal_code << std::endl;
            // otherwise: new color is "illegal"
            } else {
                new_color = fcpp::color(fcpp::coordination::illegal_color);
            }
            change_color(CALL, new_color);
        }

        //! @brief Manage state machine when robot reported an UNKNOWN error
        FUN void manage_unknown_goal_status(ARGS, RobotStatus rs) { CODE
            fcpp::color new_color;
            // and if process node is SELECTED, then AP has selected new node and simulation is ready to start    
            if (node.storage(node_process_status{}) == ProcessingStatus::SELECTED) {
                new_color = fcpp::color(fcpp::coordination::running_color);
            } else {
                new_color = fcpp::color(fcpp::coordination::idle_color);
            }
            if (feedback::GoalStatus::UNKNOWN != node.storage(node_ext_goal_status{})) {
                // update time 
                update_last_goal_update_time(CALL, node.storage(node_process_goal{}), feedback::GoalStatus::UNKNOWN);
            }
            change_color(CALL, new_color);
        }

        //! @brief Manage state machine when robot reported an abort of the current goal
        FUN void manage_aborted_goal_status(ARGS, RobotStatus rs) { CODE
            fcpp::color new_color;
            // and if process node is SELECTED, then AP has selected new node and simulation is ready to start    
            if (node.storage(node_process_status{}) == ProcessingStatus::SELECTED) {
                new_color = fcpp::color(fcpp::coordination::running_color);
            }
            else {         
                new_color = fcpp::color(fcpp::coordination::aborted_goal_color);
            }
            if (feedback::GoalStatus::ABORTED != node.storage(node_ext_goal_status{})) {
                // update time 
                update_last_goal_update_time(CALL, node.storage(node_process_goal{}), feedback::GoalStatus::ABORTED);
            }
            change_color(CALL, new_color);
        }

        //! @brief Manage state machine when robot reported a FAILED error
        FUN void manage_failed_goal_status(ARGS, RobotStatus rs) { CODE
            fcpp::color new_color;
            // and previous state is not FAILED (first time of FAILED status): change internal status to IDLE
            if (feedback::GoalStatus::FAILED != node.storage(node_ext_goal_status{})) {
                new_color = fcpp::color(fcpp::coordination::failed_goal_color);
                // resetting processing status
                node.storage(node_process_status{}) = ProcessingStatus::IDLE;
                // update time  
                update_last_goal_update_time(CALL, node.storage(node_process_goal{}), feedback::GoalStatus::FAILED);

            // or previous state is FAILED (it's NOT the first time of FAILED status)
            } else {
                // and if process node is SELECTED, then AP has selected new node and simulation is ready to start    
                if (node.storage(node_process_status{}) == ProcessingStatus::SELECTED) {
                    new_color = fcpp::color(fcpp::coordination::running_color);
                } else {
                    new_color = fcpp::color(fcpp::coordination::failed_goal_color);
                }
            }
            change_color(CALL, new_color);
        }

        //! @brief Manage state machine when robot has not yet executed a goal
        FUN void manage_no_goal_status(ARGS, RobotStatus rs) { CODE
            fcpp::color new_color;
            // and if process node is SELECTED, then AP has selected new node and simulation is ready to start    
            if (node.storage(node_process_status{}) == ProcessingStatus::SELECTED) {
                new_color = fcpp::color(fcpp::coordination::running_color);
            } else {
                new_color = fcpp::color(fcpp::coordination::idle_color);
            }
            change_color(CALL, new_color);
        }

        // AP PROCESS

        //! @brief A robot has reached a goal and now try to terminate the process
        FUN rank_data_type ends_processed_goal(ARGS,
            real_t current_rank, int current_leader_for_round,
            node_type nt, goal_tuple_type const& g, status* s) { CODE
                
                std::cout << "Robot " << node.uid << " is trying to terminate goal " <<
                  common::get<goal_code>(g) << endl;
                std::cout << endl;
                
                *s = status::terminated_output; // stop propagation
                return make_tuple(
                    current_rank, node.uid, current_leader_for_round
                );
        }

        //! @brief A robot has discharged the battery, so AP send a stop command
        FUN void battery_discharged_when_it_is_running(ARGS, goal_tuple_type const& g, status* s) { CODE
            send_stop_command_to_robot(CALL, "ABORT", node.uid, g, ProcessingStatus::IDLE);

            *s = status::border; // listen neighbours, but not send messages
        }

        //! @brief Update the counter of how many times a robot is the leader
        // TODO: at the moment, the function re-use current_leader_for_round to set the updated value.
        FUN void update_leader_counter(ARGS, float percent_charge, real_t current_rank, device_t current_leader, int& current_leader_for_round, node_type nt, goal_tuple_type const& g, status* s) { CODE
            if (AP_ENGINE_DEBUG) {
                std::cout << "Using rank received from neighbours for " << common::get<goal_code>(g)  << ": (" << current_rank << "; " << current_leader << ")"  << endl;
            }
            if (nt == node_type::ROBOT && //i'm robot
                current_rank != INF && //i have computed something
                percent_charge > 0.0 && // i have sufficient battery
                node.uid == current_leader){ //i'm the best!

                // increment leader counter because i'm selected
                current_leader_for_round++;
                std::cout << "Node with id " << node.uid << " selected " <<
                  common::get<goal_code>(g)  << " for " << current_leader_for_round << " round " << endl;
            } else {
                // resetting leader counter because i'm not selected
                current_leader_for_round = 0;
            }
        }

        //! @brief Send a GOAL action to selected node and update the AP state machine of the robot to SELECTED
        FUN void send_action_to_selected_node(ARGS, device_t current_leader, goal_tuple_type const& g) { CODE
            std::string robot_chosen = get_real_robot_name(CALL, current_leader); 
            std::cout << "Robot "   << robot_chosen  << " is chosen for goal " << common::get<goal_code>(g) << endl;
            std::cout << endl;

            // set processing status to SELECTED
            node.storage(node_process_status{}) = ProcessingStatus::SELECTED;

            // save goal
            node.storage(node_process_goal{}) = common::get<goal_code>(g);

            std::cout << "Step sent to robot: " << common::get<goal_step>(g) << endl;

            // send action to file
            action::ActionData action_data = {
                .action = common::get<goal_action>(g),
                .goal_code = common::get<goal_code>(g),
                .robot = robot_chosen,
                .pos_start_x = common::get<goal_pos_start_x>(g),
                .pos_start_y = common::get<goal_pos_start_y>(g),
                .orient_start_w = common::get<goal_orient_start_w>(g),
                .pos_end_x = common::get<goal_pos_end_x>(g),
                .pos_end_y = common::get<goal_pos_end_y>(g),
                .orient_end_w = common::get<goal_orient_end_w>(g),
                .goal_step  = common::get<goal_step>(g)
            };
            action::manager::ActionManager::new_action(action_data);
        }

        // ACTION

        //! @brief Manage when the user has requested an ABORT of a goal
        FUN void manage_action_abort(ARGS, goal_tuple_type const& g, status* s) { CODE
            std::cout << "Process ABORT "   << common::get<goal_code>(g) << " in node " << node.uid << " with status " << node.storage(node_process_status{}) << endl;
            // if and only if robot is in status RUNNING, sends stop command to robot
            if (node.storage(node_process_status{}) == ProcessingStatus::SELECTED) {
                send_stop_command_to_robot(CALL, "ABORT", node.uid, g, ProcessingStatus::TERMINATING);

                std::cout << "Robot "   << node.uid  << " is trying to terminate ABORT " << common::get<goal_code>(g) << endl;
		        *s = status::terminated_output; // stop propagation
            } else {
                if (AP_ENGINE_DEBUG) {
                    std::cout << "Robot "   << node.uid  << " is waiting to terminate ABORT " << common::get<goal_code>(g) << endl;
                }
            }
        }

        //! @brief Manage when the user has requested a new GOAL
        FUN void manage_action_goal(ARGS, node_type nt, goal_tuple_type const& g, status* s) { CODE
            
            add_goal_to_computing_map(CALL, g);
            // make_tuple(real_t, device_t, int, make_tuple(device_t, int)):
            // [0] -> rank of current leader
            // [1] -> node uid of current leader
            // [2] -> counter of how many rounds the leader is
            old(CALL, make_tuple(INF, node.uid, 0), [&] (rank_data_type current_rank_tuple) {

                // retrieve data collected from previous round
                real_t current_rank = get<0>(current_rank_tuple);
                device_t current_leader = get<1>(current_rank_tuple);
                int current_leader_for_round = int(get<2>(current_rank_tuple));

                // compute charge of battery in percent
                float percent_charge = node.storage(node_battery_charge{})/100.0;
                
                if (nt == node_type::ROBOT) {//i'm robot

                    // if i'm terminating the current goal, i have to terminate goal for all nodes
                    if (common::get<goal_code>(g) == node.storage(node_process_goal{}) && //i was running current goal in the process
                        common::get<goal_code>(g) == node.storage(node_ext_goal{}) && //the robot was running current goal
                        ProcessingStatus::TERMINATING == node.storage(node_process_status{})) { //but now i'm terminating
                        return ends_processed_goal(CALL, current_rank, current_leader_for_round, nt, g, s);
                    } 

                    // if battery is empty, then stop at current position
                    if (node.storage(node_ext_goal_status{}) == feedback::GoalStatus::RUNNING && //i'm reaching the goal
                        percent_charge <= 0.0) { //the battery is full discharged
                        battery_discharged_when_it_is_running(CALL, g, s);
                    }
                }

                update_leader_counter(CALL, percent_charge, current_rank, current_leader, current_leader_for_round, nt, g, s);
                
                // compute rank for this node
                real_t computed_rank = INF;
                computed_rank = run_rank_node(CALL, percent_charge, nt, g);   


                if (nt == node_type::ROBOT && //i'm robot
                    current_rank != INF && //i have computed something
                    computed_rank != INF && //and also now i have computed something
                    node.storage(node_process_status{}) == ProcessingStatus::IDLE && //i'm IDLE, so ready to go!
                    percent_charge > 0.0 && //i have sufficient battery
                    node.uid == current_leader && //i'm the best!
                    current_leader_for_round >= 2) // for at least 2 round
                    { 

                    send_action_to_selected_node(CALL, current_leader, g);
                }

                // blinking colors if not running
                if (common::get<goal_code>(g) != node.storage(node_process_goal{}) && 
                    node.storage(node_process_status{}) != ProcessingStatus::SELECTED) {
                    blink_computing_color(CALL);
                }

                // i'm the leader!                
                return make_tuple(
                    0.0,
                    node.uid,
                    current_leader_for_round
                );
            });
        }

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

        //! @brief Read new goals from shared variable and insert them in NewGoalsList
        FUN void read_new_goals(ARGS, std::vector<goal_tuple_type>& NewGoalsList) {
            std::lock_guard lgg(GoalMutex);
            auto map_op = [](InputGoal ig) {
              return common::make_tagged_tuple<goal_action, goal_code,
                goal_pos_start_x, goal_pos_start_y, goal_orient_start_w,
                goal_pos_end_x, goal_pos_end_y, goal_orient_end_w,
                goal_source, goal_priority, goal_subcode>(
                    ig.action,
                    ig.goal_code,
                    ig.pos_start_x,
                    ig.pos_start_y,
                    ig.orient_start_w,
                    ig.pos_end_x,
                    ig.pos_end_y,
                    ig.orient_end_w,
                    ig.source,
                    ig.priority,
                    ig.subcode
                    );
            };
            std::transform(InputGoalList.begin(), InputGoalList.end(), std::back_inserter(NewGoalsList), map_op);
            InputGoalList.clear();
        }

        // MAIN FUNCTIONS

        //! @brief Initialize MAIN function, selecting correct node_type
        FUN node_type init_main_fn(ARGS) {
            node_type nt;

            if (AP_ENGINE_DEBUG) {
              std::cout << std::endl << std::endl;
              std::cout << "[node-" << node.uid << "] Time: " <<
                std::chrono::time_point_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now()).time_since_epoch(
                    ).count() << endl;
            }

            if (node.uid == 0) {
                nt = node_type::KIOSK;
            } else if (node.uid <= ROBOTS.size()) {
                nt = node_type::ROBOT;
            }
            if (AP_ENGINE_DEBUG) {
                std::cout << "MAIN FUNCTION in node " << node.uid << " of type " << nt << endl;
            }

            return nt;
        }


        //! @brief Acquire new goals from storage, in according with node_type.
        FUN void acquire_new_goals(ARGS, node_type nt, std::vector<goal_tuple_type>& NewGoalsList) {
            if (nt == node_type::KIOSK) {
                read_new_goals(CALL, NewGoalsList);
            }
        }

        //! @brief Initialize variables (storage, etc...) of a kiosk.
        FUN void init_kiosk(ARGS) {
            // init settings: move to 0.0.0
            node.position() = make_vec(0,0,0); 
            node.storage(node_shape{}) = shape::cube;
            change_color(CALL, fcpp::color(RED));
            node.storage(node_size{})  = 0.1;
        }

        //! @brief Initialize variables (storage, etc...) of a robot using feedback data.
        FUN void init_robot(ARGS) {
            // init settings
            if (node.storage(node_size{}) == 0) {
                node.storage(node_size{}) = NODE_SIZE;
            }
            if (node.storage(node_label_size{}) == 0) {
                node.storage(node_label_size{}) = LABEL_SIZE;
            }
            node.storage(node_label_text{})   = "R."+std::to_string(node.uid);
            node.storage(node_shape{})        = shape::sphere;
            node.storage(node_label_color{})  = fcpp::color(fcpp::BLACK);
            node.storage(node_shadow_color{}) = fcpp::color(0x837E7CFF);
            node.storage(node_shadow_shape{}) = shape::sphere;

            old(CALL, robot_phase::IDLE, [&](robot_phase ph){
                std::string rname = ROBOT_PREFIX + std::to_string(node.uid); // name is obtained from node ID
                node.storage(node_ext_name{}) = rname;

                std::lock_guard lgr(RobotStatesMutex);
                if (RobotStatesMap.find(rname) != RobotStatesMap.end()) {
                    for(RobotStatus rs : RobotStatesMap[rname]) {
                        // add offset to match with background
                        real_t pos_x_offset = node.storage(node_offset_pos_x{});
                        real_t pos_y_offset = node.storage(node_offset_pos_y{});
                        node.position() = make_vec(rs.pos_x+pos_x_offset,rs.pos_y+pos_y_offset,0);

                        // truncate percentage to simply the output: 98 -> 90; 92 -> 90; 85 -> 80
                        float battery_percent_charge_trunc = std::trunc(rs.battery_percent_charge/10)*10;
                        if (rs.battery_percent_charge >= 0) {
                            // resize the node according with the battery percent charge
                            node.storage(node_size{}) = (battery_percent_charge_trunc/100.0)*NODE_SIZE;
                            node.storage(node_shadow_size{})  = (battery_percent_charge_trunc/100.0)*NODE_SHADOW_SIZE;
                        } else {
                            node.storage(node_size{}) = NODE_SIZE/2;
                            node.storage(node_shadow_size{})  = NODE_SHADOW_SIZE/2;
                        }

                        bool is_running = feedback::GoalStatus::RUNNING == rs.goal_status;
                        bool is_reached = feedback::GoalStatus::REACHED == rs.goal_status;
                        bool is_unknown = feedback::GoalStatus::UNKNOWN == rs.goal_status;
                        bool is_failed  = feedback::GoalStatus::FAILED == rs.goal_status;
                        bool is_aborted = feedback::GoalStatus::ABORTED == rs.goal_status;
                        bool is_none    = feedback::GoalStatus::NO_GOAL == rs.goal_status;
                        bool is_illegal = feedback::GoalStatus::ILLEGAL == rs.goal_status;
                        bool is_docking = feedback::DockStatus::RUNNING == rs.dock_status;
                        // change colour according with feedback from robots
                        fcpp::color new_color = fcpp::color(fcpp::coordination::idle_color);
                        // if new state is RUNNING: change to "running color"
                        if (is_running) {
                            manage_running_goal_status(CALL, rs);
                        } 
                        // if new state is REACHED:
                        else if (is_reached) {
                            manage_reached_goal_status(CALL, rs);
                            if (is_docking) {
                                manage_running_dock_status(CALL, rs);
                            }
                        }
                        // if new state is ILLEGAL: new color is illegal
                        else if (is_illegal) {
                            manage_illegal_goal_status(CALL, rs);
                        }
                        // if new state is UNKNOWN: new color is idle    
                        else if (is_unknown) {
                            manage_unknown_goal_status(CALL, rs);
                        }
                        // if new state is FAILED: new color is failed        
                        else if (is_failed) {
                            manage_failed_goal_status(CALL, rs);
                        } 
                        // if new state is ABORTED: new color is aborted        
                        else if (is_aborted) {
                            manage_aborted_goal_status(CALL, rs);
                        }
                        // if new state is NO GOAL: new color is idle        
                        else if (is_none) {
                            manage_no_goal_status(CALL, rs);
                        }
                        // otherwise: maintain previous
                        else {
                            change_color(CALL,node.storage(node_color{}));
                        }

                        // update external status in storage
                        node.storage(node_ext_goal_status{})        = feedback::GoalStatus(rs.goal_status);
                        node.storage(node_ext_docking_status{})     = feedback::DockStatus(rs.dock_status);
                        node.storage(node_ext_system_status{})      = feedback::SystemStatus(rs.system_status);
                        // update external goal in storage
                        node.storage(node_ext_goal{})               = rs.goal_code;
                        node.storage(node_ext_goal_current_step{})  = rs.goal_current_step;
                        node.storage(node_battery_charge{})         = battery_percent_charge_trunc;

                        // if battery is full discharged, change color to discharged
                        if (battery_percent_charge_trunc <= 0) {
                            manage_battery_discharged_node(CALL, rs);
                        }
                    }
                    // delete element from map
                    RobotStatesMap.erase(rname);
                }

                return ph;
            });
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
                    std::cout << "Process with code " << node.storage(node_process_goal{}) << " not found, so move robot to IDLE" << endl;
                    node.storage(node_process_status{}) = ProcessingStatus::IDLE;
                }
            }

            // search on results if the computing processes has been terminated: 
            //  - if it's terminated (or in other words, if the goal it's not in the "r" variable), we delete it from the map in the storage
            std::vector<std::string> goals_to_remove = {};
            for (auto const& x : node.storage(node_process_computing_goals{}))
            {
                auto g = x.second;
                if (r.find(g) == r.end()){
                    // std::cout << "Remove process with code " << common::get<goal_code>(g) << endl;
                    goals_to_remove.push_back(common::get<goal_code>(g));
                } 
            }
            for (auto const& gc : goals_to_remove) {
                remove_goal_from_computing_map(CALL, gc);
            }
        }

        //! @brief Main case study function.
        MAIN()
        {
            // INITIALIZE VARS
            std::vector<goal_tuple_type> NewGoalsList{};

            node_type nt = init_main_fn(CALL);

            // UPDATE DATA
            acquire_new_goals(CALL, nt, NewGoalsList);  

            // INITIALIZE NODE AND STORAGE USING NEW DATA
            if (nt == node_type::KIOSK)
            {
                init_kiosk(CALL);
            }
            else if (nt == node_type::ROBOT)
            {
                init_robot(CALL);
            }

            // PROCESS MANAGEMENT
            PROCESS(
                // NEW GOALS
                NewGoalsList, 
                // RUN FOR EACH GOAL
                if (ABORT_ACTION == common::get<goal_action>(g) && 
                    node.storage(node_process_goal{}) == common::get<goal_code>(g) && 
                    node.storage(node_ext_goal{}) == common::get<goal_code>(g)) { 
                    manage_action_abort(CALL, g, &s); 
                } else if (GOAL_ACTION == common::get<goal_action>(g)){ 
                    manage_action_goal(CALL, nt, g, &s); 
                } 
            )            
        }

        //! @brief Export types used by the *_connection functions.
        FUN_EXPORT any_connection_t = export_list<
            int,
            bool,
            goal_tuple_type,
            robot_phase,
            rank_data_type,
            feedback::GoalStatus
        >;

        //! @brief Export types used by the main function (update it when expanding the program).
        struct main_t : public export_list<
            any_connection_t,
            spawn_t<goal_tuple_type, status>, 
            diameter_election_t<tuple<real_t, device_t>>,
            nbr_t<tuple<real_t, device_t>>
        > {};

    }
}

#endif // NODES_AP_ENGINE_H_
