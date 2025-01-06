import rclpy
from rclpy.node import Node
from nav2_msgs.action import DockRobot
from geometry_msgs.msg import PoseStamped

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.client = self.create_client(DockRobot, '/dock_robot')

    def dock_robot(self, dock_id):
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        self.client.wait_for_server()
        self.client.send_goal_async(goal)

def main():
    rclpy.init()
    node = DockingNode()
    node.dock_robot('home_dock')
    rclpy.spin(node)

if __name__ == '__main__':
    main()