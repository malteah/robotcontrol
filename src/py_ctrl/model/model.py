from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse

@dataclass
class Model(object):
    initial_state: State
    operations: Dict[str, Operation]
    transitions: List[Transition]

    def __post_init__(self):
        ops = {o: "i" for o in self.operations}
        self.initial_state = self.initial_state.next(**ops)

g = predicates.guards.from_str
a = predicates.actions.from_str

# this is maybe the simplest operation, to make the rrobot robot to the buffer position
    #
    # the name of the variables and values must be unique and can not be the same as any other operatio
    # This name will be part of the state to track if the
    # operation is executing (op_move_to_buffer == "e") or not (op_move_to_buffer == "i"). The operations
    # do not have a finished state. Changing this state is already handled inside the operation
    # so you do not need to do it here
    #
    # the precondition defined when the operation is allowed to start and in this case op_move_to_buffer
    # can start when the robot is not at the buffer position. For other motions, read about the 
    # requirements in the in the assignment.
    # When the runner starts the operation, it will call start(state) on the operaiton that will
    # call next(state) on the precondition transition. This will set the command variable robot_goal_frame
    # to buffer so that the robot will go buffer. Since controlling this simulator is done with ROS actions, 
    # you also have to set and reset the robot_run variables accordingly. You can also add more actions here if for example
    # you need to block other operation to pre-start. You will read more about pre-start in the assignment handout.
    #
    # the postcondition defines when the operation has completed by checking the measured variables. This 
    # will not be possible when we are planning, since we do not have the real system during planning, so when
    # planning, we just skip checking these guards. Therefore you should only have measured variables in the guard.
    # However, in the action of the postcondition you should update your estimated variables that you 
    # expect from the operation and that you did not change when you started the operation. For example, its
    # in these actions that you update where the cubes are. In this case, we don't have position feedback
    # from the robot i.e. we will control it as an industrial robot, which means that we have to estimate its position
    # based on the executed operations. 
    #
    #The effects are used to emulate changes of measured variables when planning. For example, when calling the planner,
    # the current state of measured variables might not allow for a plan to be found. Nevertheless, the measured variables 
    # can change during the execution of the plan, so that change is emulated here to trick the planner.

def the_model() -> Model:

    initial_state = State(
        # control variables - controls the robot and gripper
        robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        robot_command = 'move_j',
        robot_velocity = 2.0,
        robot_acceleration = 0.5,
        robot_goal_frame = 'unknown',   # where to go with the tool tcp
        robot_tcp_frame = 'r1_svt_tcp', # the tool tcp to use

        gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        gripper_command = 'none', # pick_red, pick_green, pick_blue, place_red, place_green, place_blue
        robot_pose = "unknown",
        goal_as_string = "",
        replan = False,

        # measured variables - tracks the state of the robot
        robot_state = "initial",  # "exec", "done", "failed" 
        replanned = False,

        #estimated - tracks the cubes position
        green_cube_at = "pose_1", # pose_1, pose_2, pose_3, gripper, buffer
        red_cube_at = "pose_2",  # pose_1, pose_2, pose_3, gripper, buffer
        blue_cube_at = "pose_3",  # pose_1, pose_2, pose_3, gripper, buffer


        
    )

    # we will store all operations in this dict that will be part of the model
    ops = {}


    
    for pose in ["pose_1", "pose_2", "pose_3", "buffer"]: # "buffer"
        for col in ["red","green","blue"]:

            ops[f"op_move_to_{pose}"] = Operation(
                name=f"op_move_to_{pose}",

                precondition=Transition("pre", 
                g(f"robot_pose == above_{pose} && robot_state == initial && !robot_run"), 
                a(f"robot_command = move_j, robot_run, robot_goal_frame = {pose}" )),

                postcondition=Transition("post", 
                g(f"robot_state == done"), 
                a(f"!robot_run, robot_pose <- {pose}")),

                effects= a(f"robot_state = initial"), # a(f"robot_pose <- {pose}")

                to_run = Transition.default()
            )
            ops[f"op_move_to_above_{pose}"] = Operation(
                name=f"op_move_to_above_{pose}",
                
                precondition=Transition("pre",
                g(f"robot_pose != above_{pose} && robot_state == initial && !robot_run"),
                a(f"robot_command = move_j, robot_run, robot_goal_frame = above_{pose}" )),

                postcondition=Transition("post", 
                g(f"robot_state == done"), 
                a(f"!robot_run, robot_pose = above_{pose}")),
                effects= a(f"robot_state = initial"),

                to_run = Transition.default()
            )

            ops[f"op_pick_up_{col}_from_{pose}"] = Operation(
                name=f"op_pick_up_{col}_from_{pose}",

                precondition=Transition("pre", 
                g(f"robot_pose == {pose} && {col}_cube_at == {pose} && !gripper_run && !robot_run && red_cube_at != gripper && blue_cube_at != gripper && green_cube_at != gripper"), #&& !gripper_run && robot_state == initial && !robot_run
                a(f"gripper_run, gripper_command = pick_{col}")),

                postcondition=Transition("post", 
                g(f"gripper_command == done"), #gripper_run == False, 
                a(f"!gripper_run,  {col}_cube_at = gripper")), #robot_goal_frame = above_{pose}

                effects= a(f"robot_state = initial"), 

                to_run = Transition.default()
            )

            ops[f"op_place_{col}_at_{pose}"] = Operation(
                name=f"op_place_{col}_at_{pose}",

                precondition=Transition("pre", 
                g(f"robot_pose == {pose} && {col}_cube_at == gripper && red_cube_at != {pose} && blue_cube_at != {pose} && green_cube_at != {pose}"), 
                a(f"gripper_run, gripper_command = place_{col}")), #robot_goal_frame = {pose}

                postcondition=Transition("post", 
                g(f"gripper_command == done"), #gripper_run == False
                a(f"!gripper_run, {col}_cube_at = {pose}")), #robot_goal_frame = above_{pose}, robot_pose = above_{pose}, !robot_run, 

                effects= a(f"robot_state = initial"), 

                to_run = Transition.default()
            )

            """ops[f"op_pick_up_{col}"] = Operation(
                name=f"op_pick_up_{col}",

                precondition=Transition("pre", 
                g(f"robot_pose == above_{pose} && {col}_cube_at == {pose} && robot_state == initial && !robot_run"), 
                a(f"gripper_run, gripper_command = pick_{col}")), #robot_goal_frame = {pose}

                postcondition=Transition("post", 
                g(f"robot_state == done"), #gripper_state == exec?
                a(f"!robot_run, robot_pose = above_{pose}, {col}_cube_at = gripper")), #robot_goal_frame = above_{pose}

                effects= (), # a(f"robot_pose <- {pose}")

                to_run = Transition.default()
            )"""

            """ops[f"op_place_{col}"] = Operation(
                name=f"op_place_{col}",

                precondition=Transition("pre", 
                g(f"robot_pose == above_{pose} && robot_state == initial && !robot_run"), 
                a(f"robot_command = move_j, robot_run, gripper_run, gripper_command = place_{col}, robot_goal_frame = {pose}" )),

                postcondition=Transition("post", 
                g(f"robot_state == done && !gripper_run"), #gripper_state == exec?
                a(f"!robot_run, robot_goal_frame = above_{pose}")),
                effects= (), # a(f"robot_pose <- {pose}")

                to_run = Transition.default()
            )"""
    """
            ops[f"op_pick_{col}"] = Operation(
                name=f"op_pick_{col}", 

                precondition=Transition("pre", 
                g(f"robot_state == initial && !robot_run && robot_pose == {pose} && !gripper_run && gripper_command == pick_{col} && {col}_cube_at == {pose}"), 
                a(f"gripper_command = pick_{col}, gripper_run, ")),

                postcondition=Transition("post",
                g(f"gripper_run,  "), 
                a(f"gripper_command = none")),

                effects= (),

                to_run = Transition.default()
            )


            ops[f"op_move_to_{col}"] = Operation(
                name=f"op_moce_to_{col}",

                precondition=Transition("pre", 
                g(f"robot_pose != {pose} && robot_state == initial && !robot_run"), 
                a(f"robot_command = move_j, robot_run, robot_goal_frame = {pose}" )),

                postcondition=Transition("post", 
                g(f"robot_state == done"), 
                a(f"!robot_run, robot_pose <- {pose}")),

                effects= (), # a(f"robot_pose <- {pose}")

                to_run = Transition.default()
            )
    """
   
    transitions: List[Transition] = []

    return Model(
        initial_state,
        ops,
        transitions
    )

def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    
    pos1: str = cube_state.pos1
    pos2: str = cube_state.pos2
    pos3: str = cube_state.pos3
    goal = g(f"pos1_col == {pos1} && pos2_col == {pos2} && pos3_col)
    return goal

    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()
