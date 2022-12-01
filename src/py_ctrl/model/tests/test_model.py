import pytest
from predicates.state import State
from predicates import guards, actions
from model.operation import Transition, Operation
from predicates.errors import NextException
from model.model import Model, the_model

def test_model_creation():
    m = the_model()
    enabled_in_initial = [o for name, o in m.operations.items() if o.eval(m.initial_state)]
    assert len(enabled_in_initial) > 0

# write your own tests so that you know that the model you have created is the one you expected.
# for example, write a test for each operation so that it is enabled in the correct states and
# that it changes the state both when using next_planning and start and complete

def test_op_move_to_buffer():
    m = the_model()
    ops = m.operations

    test_state = m.initial_state.next(robot_pose = "pose_1")
    o = ops["op_move_to_buffer"]
    
    after_start = o.start(test_state)
    not_completed = o.is_completed(after_start)
    completed = o.is_completed(after_start.next(robot_goal_frame = "buffer"))
    after_completed = o.complete(after_start.next(robot_goal_frame = "buffer"))
    
    assert o.eval(test_state)
    assert after_start == test_state.next(robot_pose = "buffer", op_move_to_buffer = "e")
    assert not not_completed
    assert completed
    assert after_completed == after_start.next(robot_goal_frame = "buffer", op_move_to_buffer = "i")
    
    after_planned = o.next_planning(test_state)
    assert after_planned == test_state.next(robot_pose = "buffer", robot_goal_frame = "buffer")


def test_some_operations(): 
    m = the_model()
    s = m.initial_state
    at_p1 = m.operations["op_move_to_pose_1"].next_planning(s)
    assert guards.Eq("robot_goal_frame", "pose_1").eval(at_p1)

# Write your own tests to check the all the operations 

# Write your own tests to check the all the operations 
def test_red_cube_at_buffer():
    m = the_model()
    s = m.initial_state
    o1 = m.operations["op_move_to_above_pose_2"]
    o2 = m.operations["op_move_to_pose_2"]
    o3 = m.operations["op_pick_up_red_from_pose_1"]
    o4 = m.operations["op_move_to_above_buffer"]
    o5 = m.operations["op_move_to_buffer"]
    o6 = m.operations["robot_place_red_at_buffer"]
    new_state = s
    for op in [o1, o2, o3, o4, o5, o6]:
        if op.eval(new_state):
            new_state = op.next_planning(new_state)
        else:
            assert False
    print(new_state)
    assert new_state.get("red_cube_at") == "buffer" 

# write your own tests so that you know that the model you have created is the one you expected.
# for example, write a test for each operation so that it is enabled in the correct states and
# that it changes the state both when using next_planning and start and complete