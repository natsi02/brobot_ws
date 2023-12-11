#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String 
from std_msgs.msg import Int8
# from brobot_speak_logic import brobot_speak_logic 
from brobot_interfaces.srv import SpeakTrigger
from brobot_interfaces.srv import ListenTrigger
from brobot_hand_logic import brobot_hand_logic
class brobot_interact(Node): 
    def __init__(self):
        super().__init__('interact_node')
        self.camera_detect = None
        self.hand_detect = None
        self.username = None 
        self.listen_message = None 
        self.is_speaking = False
        self.is_listening = False
        self.is_interact = False

        self.create_subscription(Bool, "/camera_detect", self.camera_detect_callback, 10)
        self.create_subscription(Int8, "/hand_detect", self.hand_detect_callback,  10)
        self.create_subscription(String, "/username", self.get_user_name_callback,  10)
        
        self.speak_client = self.create_client(SpeakTrigger, '/trigger_speak')
        self.listen_client = self.create_client(ListenTrigger, '/trigger_listen' )

        while not self.speak_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Speak Service not available, waiting again...')


        while not self.listen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Listen Service not available, waiting again...')


        self.speak_req = SpeakTrigger.Request()
        self.listen_req = ListenTrigger.Request()


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
        if (hand_action == 1 and self.is_speaking == False):
            hand_action = self.bb_hand_logic.add_hand_data(0)
            self.speak_req.status = 1
            self.trigger_speak()
            self.hand_detect = 0
        if (hand_action == 2 and self.is_listening == False):
            hand_action = self.bb_hand_logic.add_hand_data(0)
            self.speak_req.status = 0
            self.speak_req.message = "Hey, This is me brobot, anything I can help you"
            self.trigger_speak()
            hand_action = self.bb_hand_logic.add_hand_data(0)
            self.trigger_listen()
            self.hand_detect = 0




    def speak_callback(self, future):
        self.get_logger().info(f'>>>>>>>>>>>>>>>: {future.result().success} ' ) 
        self.is_speaking = False

    def listen_callback(self, future):
        self.get_logger().info(f'>>>>>>>>>>>>>>>: {future.result().status} ' ) 
        self.get_logger().info(f'>>>>>>>>>>>>>>>: {future.result().message} ' ) 
        self.listen_message  = str(future.result().message)
        self.is_listening = False

        self.speak_req.message = self.listen_message
        self.trigger_speak()


    def trigger_speak(self):
        self.is_speaking = True
        self.future = self.speak_client.call_async(self.speak_req)
        self.future.add_done_callback(self.speak_callback)    

    
    def trigger_listen(self):
        self.is_listening = True
        self.future = self.listen_client.call_async(self.listen_req)
        self.future.add_done_callback(self.listen_callback)    
     

def main(args=None):
    rclpy.init(args=args)
    node = brobot_interact() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
