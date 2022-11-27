import rclpy
import json
from std_msgs.msg import String
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_goal')  # type: ignore

    publisher = node.create_publisher(String, '/set_state', 10)

    msg = String()
    pos = {"replan": False}

    msg.data = json.dumps(pos)
    publisher.publish(msg)

    time.sleep(0.2)

    # set a state here to force replan
    pos = {
        "replan": True, 
        # change the goal here
        "goal_as_string": "green_cube_at == gripper && robot_pose == above_pose_1" # robot_pose == above_pose_2 , green_cube_at == gripper
    }

    msg.data = json.dumps(pos)
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()