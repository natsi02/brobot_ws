#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String 
from std_msgs.msg import Int8

from brobot_hand_logic import brobot_hand_logic

from brobot_interfaces.action import SpeakAction
from brobot_interfaces.action import ListenAction
from rclpy.action import ActionClient
class brobot_interact(Node): 
    def __init__(self):
        super().__init__('interact_node')
        self.camera_detect = None
        self.hand_detect = None
        self.username = None 
        self.listen_message = None 
        self.is_speaking = False
        self.is_interact = False

        self.create_subscription(Bool, "/camera_detect", self.camera_detect_callback, 10)
        self.create_subscription(Int8, "/hand_detect", self.hand_detect_callback,  10)
        self.create_subscription(String, "/username", self.get_user_name_callback,  10)
        
        self._speak_action_client = ActionClient(self, SpeakAction, '/speak_action')
        self._listen_action_client = ActionClient(self, ListenAction, '/listen_action')

        self.bb_hand_logic = brobot_hand_logic()

    def get_user_name_callback(self, msg):
        self.username = msg.data
        # self.get_logger().info(f'Get Username Data: {self.username } ' ) 

    def camera_detect_callback(self, msg):
        self.camera_detect = msg.data
        # self.get_logger().info(f'Get Face Detect Data: {self.camera_detect } ' ) 

    def hand_detect_callback(self, msg): # interaction occurs here 
        self.hand_detect = msg.data
        self.get_logger().info(f'Get Hand Detect Data: {self.hand_detect} ' ) 
        hand_action = self.bb_hand_logic.add_hand_data(self.hand_detect)
        self.get_logger().info(f'------------------->: {hand_action, self.is_speaking} ' ) 
        if (hand_action == 1):
            hand_action = self.bb_hand_logic.add_hand_data(0)
            status1 = 1
            talk1 = None
            self.send_speak_action(str(talk1), status1)
            self.hand_detect = 0
        if (hand_action == 2):
            hand_action = self.bb_hand_logic.add_hand_data(0)
            status1 = 0
            talk1 = "Hey, This is me brobot, anything I can help you"
            self.send_speak_action(str(talk1), status1)
            hand_action = self.bb_hand_logic.add_hand_data(0)
            self.send_listen_action()
            self.hand_detect = 0


    # --- Speak Action 

    def send_speak_action(self, message, status):
        speak_msg = SpeakAction.Goal()
        speak_msg.message = message
        speak_msg.status = status

        self._speak_action_client.wait_for_server()
        self._send_goal_future = self._speak_action_client.send_goal_async(speak_msg, feedback_callback=self.speak_feedback_callback)
        self._send_goal_future.add_done_callback(self.speak_response_callback)
    
    def speak_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.speak_result_callback)

    def speak_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.speakresult))

    def speak_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.speakfeedback))


    # --- Listen Action 

    def send_listen_action(self):
        self._listen_action_client.wait_for_server()
        self._send_goal_future = self._listen_action_client.send_goal_async(ListenAction.Goal())
        self._send_goal_future.add_done_callback(self.listen_response_callback)
    
    def listen_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.listen_result_callback)

    def listen_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result Message : {0}'.format(result.message))
        self.get_logger().info('Result Status: {0}'.format(result.status))
        self.listen_message = result.message
        if (self.listen_message != None):
            self.send_speak_action(str(self.listen_message), 0)
            self.listen_message = None


def main(args=None):
    rclpy.init(args=args)
    node = brobot_interact() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
