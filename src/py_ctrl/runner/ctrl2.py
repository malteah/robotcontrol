from functools import partial
from typing import Any, Optional, Tuple, List
import json
from predicates.state import State
from model.model import Model, from_goal_to_goal
from model.model2 import the_model
from planner.plan import plan
from model.operation import Operation
from predicates.state import State
from predicates.errors import NotInStateException
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.publisher import Publisher
from ur_controller_msgs.action import URControl
from scene_manipulation_msgs.srv import ManipulateScene
from std_srvs.srv import Trigger
import random


import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

# ---------------------------------------------------------------------------
# ...
# ---------------------------------------------------------------------------


runner_goal: str = "runner_goal"
runner_plan: str = "runner_plan"
step_in_plan: str = "step_in_plan"
plan_status: str = "plan_status"

# publish a goal:

class Runner(Node):
    
    def __init__(self):
        super().__init__('the_runner')  # type: ignore
        self.model: Model = the_model()
        self.state: State = self.model.initial_state
        self.prev_state = self.state
        self.lock_waiting = False
        self.upd_state(runner_goal, None)
        self.upd_state(runner_plan, None)
        self.upd_state(step_in_plan, None)
        self.upd_state(plan_status, None)

        self.r1_ur_robot_action_client = ActionClient(self, URControl, '/r1/ur_control')
        self.r1_robot_action_goal_handle: Optional[ClientGoalHandle] = None

        self.r2_ur_robot_action_client = ActionClient(self, URControl, '/r2/ur_control')
        self.r2_robot_action_goal_handle: Optional[ClientGoalHandle] = None

        self.sms_client = self.create_client(ManipulateScene, "manipulate_scene")
        self.sms_request = ManipulateScene.Request()
        self.sms_response = ManipulateScene.Response()

        while not self.sms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Manipulate Scene Service not available, waiting again..."
            )

        ## We will not use the goal topic. Should be defind using the state
        self.create_subscription(
            msg_type = String,
            topic = 'set_state',
            callback = self.set_state_callback,
            qos_profile = 10)

        self.pub_state: Publisher = self.create_publisher(
            msg_type=String,
            topic = 'state',
            qos_profile = 10,
        )

        self.timer = self.create_timer(0.1, self.ticker)  # type: ignore

    def manipulate_scene(self, child, parent):
        self.sms_request.command = 'reparent'
        self.sms_request.child_frame_id = child
        self.sms_request.parent_frame_id = parent
        self.persist = False
        self.sms_future = self.sms_client.call_async(self.sms_request)
        self.sms_future.add_done_callback(partial(self.sms_future_callback))

    def sms_future_callback(self, future):
        try:
            self.sms_response = future.result()
        except Exception as e:
            self.get_logger().info("SMS service call failed %r" % (e,))     
    
    def set_state_callback(self, msg: String):
        """
        Here you can send in state changes from outside
        """
        try:
            j = msg.data.replace('\'', '\"')
            kvs: dict[str, Any] = json.loads(j)
            print(f"got a state change: {kvs}")
            self.state = self.state.next(**kvs)
        except TypeError:
            pass
        except json.decoder.JSONDecodeError as e:
            print(f"message is bad: {msg.data}")
            print(e)

    def send_sms_request(self):
        r1_run: bool = self.state.get('r1_gripper_run')
        r1_grip_cmd = self.state.get('r1_gripper_command')
        r2_run: bool = self.state.get('r2_gripper_run')
        r2_grip_cmd = self.state.get('r2_gripper_command')
        if r1_run:
            if r1_grip_cmd == "pick_red":
                self.pick_r1("red_cube", "r1_svt_tcp")
            if r1_grip_cmd == "pick_green":
                self.pick_r1("green_cube", "r1_svt_tcp")
            if r1_grip_cmd == "pick_blue":
                self.pick_r1("blue_cube", "r1_svt_tcp")
            if r1_grip_cmd == "place_red":
                self.place_r1("red_cube")
            if r1_grip_cmd == "place_green":
                self.place_r1("green_cube")
            if r1_grip_cmd == "place_blue":
                self.place_r1("blue_cube")
        else:
            self.upd_state('r1_gripper_command', "none")

        if r2_run:
            if r2_grip_cmd == "pick_red":
                self.pick_r2("red_cube", "r2_svt_tcp")
            if r2_grip_cmd == "pick_green":
                self.pick_r2("green_cube", "r2_svt_tcp")
            if r2_grip_cmd == "pick_blue":
                self.pick_r2("blue_cube", "r2_svt_tcp")
            if r2_grip_cmd == "place_red":
                self.place_r2("red_cube")
            if r2_grip_cmd == "place_green":
                self.place_r2("green_cube")
            if r2_grip_cmd == "place_blue":
                self.place_r2("blue_cube")
        else:
            self.upd_state('r2_gripper_command', "none")

    def pick_r1(self, cube, tcp):
        self.manipulate_scene(
            cube,
            tcp
        )
        if self.sms_response.success: # type: ignore
            self.upd_state('r1_gripper_command', "done")

    def place_r1(self, cube):
        self.manipulate_scene(
            cube,
            "world"
        )
        if self.sms_response.success: # type: ignore
            self.upd_state('r1_gripper_command', "done")

    def pick_r2(self, cube, tcp):
        self.manipulate_scene(
            cube,
            tcp
        )
        if self.sms_response.success: # type: ignore
            self.upd_state('r2_gripper_command', "done")

    def place_r2(self, cube):
        self.manipulate_scene(
            cube,
            "world"
        )
        if self.sms_response.success: # type: ignore
            self.upd_state('r2_gripper_command', "done")

    def r1_send_ur_action_goal(self):
        run: bool = self.state.get('r1_robot_run')
        if not run and self.r1_robot_action_goal_handle is not None:
            self.r1_robot_action_goal_handle.cancel_goal()  # maybe do it async?
            print("Cancel of robot action done")
            self.r1_robot_action_goal_handle = None
            self.upd_state('r1_robot_state', "initial")
        elif not run:
            self.upd_state('r1_robot_state', "initial")
        elif run and self.r1_robot_action_goal_handle is None: 
            print("start action")
            goal_msg = URControl.Goal()
            goal_msg.command = self.state.get('r1_robot_command')
            goal_msg.velocity = self.state.get('r1_robot_velocity')
            goal_msg.acceleration = self.state.get('r1_robot_acceleration')
            goal_msg.goal_feature_id = self.state.get('r1_robot_goal_frame')
            goal_msg.tcp_id = self.state.get('r1_robot_tcp_frame')

            print("waiting action")
            if self.r1_ur_robot_action_client.wait_for_server(2):
                print("done waiting action")
                send_goal_future = self.r1_ur_robot_action_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self.r1_ur_action_goal_response_callback)
            else:
                print("timeout action")

    def r2_send_ur_action_goal(self):
        run: bool = self.state.get('r2_robot_run')
        if not run and self.r2_robot_action_goal_handle is not None:
            self.r2_robot_action_goal_handle.cancel_goal()  # maybe do it async?
            print("Cancel of robot action done")
            self.r2_robot_action_goal_handle = None
            self.upd_state('r2_robot_state', "initial")
        elif not run:
            self.upd_state('r2_robot_state', "initial")
        elif run and self.r2_robot_action_goal_handle is None: 
            print("start action")
            goal_msg = URControl.Goal()
            goal_msg.command = self.state.get('r2_robot_command')
            goal_msg.velocity = self.state.get('r2_robot_velocity')
            goal_msg.acceleration = self.state.get('r2_robot_acceleration')
            goal_msg.goal_feature_id = self.state.get('r2_robot_goal_frame')
            goal_msg.tcp_id = self.state.get('r2_robot_tcp_frame')

            print("waiting action")
            if self.r2_ur_robot_action_client.wait_for_server(2):
                print("done waiting action")
                send_goal_future = self.r2_ur_robot_action_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self.r2_ur_action_goal_response_callback)
            else:
                print("timeout action")


    def r1_ur_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.upd_state('r1_robot_state', "failed")
            self.r1_robot_action_goal_handle = None
            return

        self.r1_robot_action_goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')
        self.upd_state('r1_robot_state', "exec")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.r1_ur_action_get_result_callback)

    def r2_ur_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.upd_state('r2_robot_state', "failed")
            self.r2_robot_action_goal_handle = None
            return

        self.r2_robot_action_goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')
        self.upd_state('r2_robot_state', "exec")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.r2_ur_action_get_result_callback)


    def r1_ur_action_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.upd_state('r1_robot_state', "done")
        self.upd_state('r1_robot_pose', self.state.get('r1_robot_goal_frame'))
        self.r1_robot_action_goal_handle = None
    
    def r2_ur_action_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.upd_state('r2_robot_state', "done")
        self.upd_state('r2_robot_pose', self.state.get('r2_robot_goal_frame'))
        self.r2_robot_action_goal_handle = None

    def upd_state(self, key: str, value):
        self.state = self.state.next(**{key: value})

    def ticker(self):
        g = self.state.get(runner_goal)
        p = self.state.get(runner_plan)
        replan = self.state.get("replan")
        replanned = self.state.get("replanned")
            
        if replan and not replanned:
            self.upd_state("replanned", True)
            goal = from_goal_to_goal(self.state)
            print(f"The goal: {goal}")
            new_p = plan(self.state, goal, self.model, 30)
            if new_p == None:
                print(f"Could not find a plan to goal: {goal}")
            elif len(new_p) == 0:
                print(f"We are already in goal: {goal}")
            else:
                self.upd_state(runner_plan, new_p)
                print(f"The new goal: {goal}")
                print(f"and computed this plan: {new_p}")
        
        if not replan:
            self.upd_state("replanned", False)


        if self.prev_state != self.state:
            print(f"")
            for k, v in self.state.items():
                print(f"{k}: {v}")
            print(f"")
    
        self.prev_state = self.state

        # here we call the ticker. Change the pre_start parameter to true when
        # you want to prestart
        self.state = tick_the_runner(self.state, self.model, True)

        # below, we are publishing the command variables to the simulation via ros
        self.r1_send_ur_action_goal()
        self.r2_send_ur_action_goal()
        self.send_sms_request()
        state_json = json.dumps(self.state.state)
        self.pub_state.publish(String(data = state_json))      


def tick_the_runner(state: State, model: Model, pre_start: bool) -> State:
    """
    This function will run the operations based on a plan that are located in the state
    This will just execute one transition at the time
    """

    # Here you can execute the free transitions by checking if they are enabled and then do next on them

    for t in model.transitions:
        if t.eval(state):
            state = t.next(state)

    the_plan: list[str] = state.get(runner_plan)    
    if not the_plan:
        return state.next(plan_status="No plan in state", runner_plan = None, step_in_plan = None)
    
    current_step_in_plan: int = state.get(step_in_plan)
    if not current_step_in_plan:
        # we have not started executing the plan so we start at position 0 in the plan
        current_step_in_plan = 0
        state = state.next(**{step_in_plan: current_step_in_plan})
    
    plan_length = len(the_plan)
    if plan_length <= current_step_in_plan:
        # we are done with the plan and will stop executing and we also
        # reset the current plan so we do not tries to run the same plan again
        return state.next(plan_status="done", runner_plan = None, step_in_plan = None)

    # check what operation we are / should be executing
    current_op_name = the_plan[current_step_in_plan]
    current_op_state: str = state.get(current_op_name)
    current_op: Operation = model.operations[current_op_name]

    next_step = current_step_in_plan + 1

    if current_op_state == "i" and current_op.eval_run(state): # The operation can be started
        next_state = current_op.start(state)
    elif current_op_state == "i": # the operation should be started but is not enabled
        next_state = state.next(plan_status=f"waiting for op {current_op_name} to be enabled. pre: {current_op.precondition} and {current_op.to_run}")
    elif current_op.is_completed(state): # the operation has completed and we can take a step in the plan
        next_state = current_op.complete(state)
        next_state = next_state.next(step_in_plan=next_step, plan_status=f"completing step {current_step_in_plan}")
    elif current_op_state == "e": # the operation is executing, let's check if we can prestart the next
        if not pre_start:
            next_state = state.next(plan_status=f"waiting for op to complete")
        elif plan_length > next_step and model.operations[the_plan[next_step]].eval_run(state):
            next_state = model.operations[the_plan[next_step]].start(state).next(
                plan_status=f"pre_starting {next_step}"
            )
        else:
            next_state = state
    else:
        next_state = state.next(plan_status="doing nothing")
    
    return next_state


def tick_the_random_runner(state: State, model: Model) -> State:
    """
    This function will run the operations based on a plan that are located in the state
    This will just execute one transition at the time
    """

    running_ops: List[Operation] = [o for name, o in model.operations.items() if state.get(name) == "e"]
    next_state = state

    if not running_ops:
        enabled_ops = [o for _, o in model.operations.items() if o.precondition.eval_run(state)]  # type: ignore
        if not enabled_ops:
            print("No operations are enabled or running in this state!")
            return state
        
        o = random.choice(enabled_ops)
        next_state = o.start(state)
        print(f"Operation {o.name} started!")

    else:
        ops_can_complete = [o for o in running_ops if o.postcondition.eval_run(state)]  # type: ignore
        if ops_can_complete:
            next_state = state
            for o in ops_can_complete:
                next_state = o.complete(next_state)
                print(f"Operation {o.name} completed!")

    return next_state


def run():
    rclpy.init()
    runner = Runner()
    rclpy.spin(runner)
    runner.destroy_node()
    rclpy.shutdown()

