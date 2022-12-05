from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse
from model.model import Model, from_goal_to_goal

g = predicates.guards.from_str
a = predicates.actions.from_str


def the_model() -> Model:

    initial_state = State(
        # control variables
        r1_robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        r1_robot_command = 'move_j',
        r1_robot_velocity = 2.0,
        r1_robot_acceleration = 0.5,
        r1_robot_goal_frame = 'unknown',   # where to go with the tool tcp
        r1_robot_tcp_frame = 'r1_svt_tcp', # the tool tcp to use
        r1_gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        r1_gripper_command = 'none', # pick_red, pick_green, pick_blue, drop_red, drop_green, drop_blue

        r2_robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        r2_robot_command = 'move_j',
        r2_robot_velocity = 2.0,
        r2_robot_acceleration = 0.5,
        r2_robot_goal_frame = 'unknown',   # where to go with the tool tcp
        r2_robot_tcp_frame = 'r2_svt_tcp', # the tool tcp to use
        r2_gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        r2_gripper_command = 'none', # pick_red, pick_green, pick_blue, drop_red, drop_green, drop_blue

        goal_as_string = "",
        replan = False,

        # measured variables
        r1_robot_state = "initial",  # "exec", "done", "failed" 
        r1_robot_pose = "unknown",
        r2_robot_state = "initial",  # "exec", "done", "failed" 
        r2_robot_pose = "unknown",
        replanned = False,

        #estimated
        green_cube_at = "pose_1", # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
        red_cube_at = "pose_2",  # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
        blue_cube_at = "pose_3",  # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
    )
    # r1_robot_tcp_frame = 'r1_svt_tcp', # the tool tcp to use
    ops = {}
    for pose in ["pose_1", "pose_2", "r1_buffer"]: # "buffer"
        for col in ["red","green","blue"]: # "green","blue"

            ops[f"op_r1_move_to_{pose}"] = Operation(
                name=f"op_r1_move_to_{pose}",

                precondition=Transition("pre", 
                g(f"r2_robot_pose != {pose} && r1_robot_pose != {pose} && r1_robot_state == initial && !r1_robot_run"), 
                a(f"r1_robot_command = move_j, r1_robot_run, r1_robot_goal_frame = {pose}" )),

                postcondition=Transition("post", 
                g(f"r1_robot_state == done"), 
                a(f"!r1_robot_run, r1_robot_pose <- {pose}")),

                effects= a(f"r1_robot_state = initial"),

                to_run = Transition.default()
            )
            
            ops[f"op_r1_pick_up_{col}_from_{pose}"] = Operation(
                name=f"op_r1_pick_up_{col}_from_{pose}",

                precondition=Transition("pre", 
                g(f"r1_robot_pose == {pose} && {col}_cube_at == {pose} && !r1_gripper_run && !r1_robot_run && red_cube_at != r1_gripper && blue_cube_at != r1_gripper && green_cube_at != r1_gripper"), #&& !gripper_run && robot_state == initial && !robot_run
                a(f"r1_gripper_run, r1_gripper_command = pick_{col}")),

                postcondition=Transition("post", 
                g(f"r1_gripper_command == done"), #gripper_run == False, 
                a(f"!r1_gripper_run,  {col}_cube_at = r1_gripper")), #robot_goal_frame = above_{pose}

                effects= a(f"r1_robot_state = initial"), 

                to_run = Transition.default()
            )

            ops[f"op_r1_place_{col}_at_{pose}"] = Operation(
                name=f"op_r1_place_{col}_at_{pose}",

                precondition=Transition("pre", 
                g(f"r1_robot_pose == {pose} && {col}_cube_at == r1_gripper && red_cube_at != {pose} && blue_cube_at != {pose} && green_cube_at != {pose}"), 
                a(f"r1_gripper_run, r1_gripper_command = place_{col}")), #robot_goal_frame = {pose}

                postcondition=Transition("post", 
                g(f"r1_gripper_command == done"), #gripper_run == False
                a(f"!r1_gripper_run, {col}_cube_at = {pose}")), #robot_goal_frame = above_{pose}, robot_pose = above_{pose}, !robot_run, 

                effects= a(f"r1_robot_state = initial"), 

                to_run = Transition.default()
            )
    
    for pose in ["pose_2", "pose_3", "r2_buffer"]: 
        for col in ["red","green","blue"]:

            ops[f"op_r2_move_to_{pose}"] = Operation(
                name=f"op_r2_move_to_{pose}",

                precondition=Transition("pre", 
                g(f"r2_robot_pose != {pose} && r1_robot_pose != {pose} && r2_robot_state == initial && !r2_robot_run"), 
                a(f"r2_robot_command = move_j, r2_robot_run, r2_robot_goal_frame = {pose}" )),

                postcondition=Transition("post", 
                g(f"r2_robot_state == done"), 
                a(f"!r2_robot_run, r2_robot_pose <- {pose}")),

                effects= a(f"r2_robot_state = initial"),

                to_run = Transition.default()
            )
            
            ops[f"op_r2_pick_up_{col}_from_{pose}"] = Operation(
                name=f"op_r2_pick_up_{col}_from_{pose}",

                precondition=Transition("pre", 
                g(f"r2_robot_pose == {pose} && {col}_cube_at == {pose} && !r2_gripper_run && !r2_robot_run && red_cube_at != r2_gripper && blue_cube_at != r2_gripper && green_cube_at != r2_gripper"), #&& !gripper_run && robot_state == initial && !robot_run
                a(f"r2_gripper_run, r2_gripper_command = pick_{col}")),

                postcondition=Transition("post", 
                g(f"r2_gripper_command == done"), #gripper_run == False, 
                a(f"!r2_gripper_run,  {col}_cube_at = r2_gripper")), #robot_goal_frame = above_{pose}

                effects= a(f"r2_robot_state = initial"), 

                to_run = Transition.default()
            )

            ops[f"op_r2_place_{col}_at_{pose}"] = Operation(
                name=f"op_r2_place_{col}_at_{pose}",

                precondition=Transition("pre", 
                g(f"r2_robot_pose == {pose} && {col}_cube_at == r2_gripper && red_cube_at != {pose} && blue_cube_at != {pose} && green_cube_at != {pose}"), 
                a(f"r2_gripper_run, r2_gripper_command = place_{col}")), #robot_goal_frame = {pose}

                postcondition=Transition("post", 
                g(f"r2_gripper_command == done"), #gripper_run == False
                a(f"!r2_gripper_run, {col}_cube_at = {pose}")), #robot_goal_frame = above_{pose}, robot_pose = above_{pose}, !robot_run, 

                effects= a(f"r2_robot_state = initial"), 

                to_run = Transition.default()
            )

    """ops[f"op_r{num}_move_to_above_{pose}"] = Operation(
                    name=f"op_r{num}_move_to_above_{pose}",
                    
                    precondition=Transition("pre",
                    g(f"r{num}_robot_pose != above_{pose} && r{o_num}_robot_pose != above_{pose} && r{o_num}_robot_pose != {pose} && r{num}_robot_state == initial && !r{num}_robot_run"),
                    a(f"r{num}_robot_command = move_j, r{num}_robot_run, r{num}_robot_goal_frame = above_{pose}" )),

                    postcondition=Transition("post", 
                    g(f"r{num}_robot_state == done"), 
                    a(f"!r{num}_robot_run, r{num}_robot_pose = above_{pose}")),

                    effects= a(f"r{num}_robot_state = initial"),

                    to_run = Transition.default()
                )"""

    transitions: List[Transition] = []

    return Model(
        initial_state,
        ops,
        transitions
    )

def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()