#!/usr/bin/python3
import pyttsx3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from brobot_interfaces.srv import SpeakTrigger

class brobot_speak(Node):
    def __init__(self):
        super().__init__('speak_node')
        self.engine = pyttsx3.init()
        self.trigger_service = self.create_service(SpeakTrigger, '/trigger_speak', self.trigger_speak)

    def bb_speak(self, command) -> None:
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[3].id) # 0 (man) | 1 (girl)

        self.engine.setProperty('rate', 100)  # Adjust the speech rate
        self.engine.setProperty('volume', 1.0)  # Adjust the volume (1.0 is full volume)

        self.engine.say(command)
        self.engine.runAndWait()
        self.get_logger().info("Received service call2")



    def trigger_speak(self, request, response):
        # response = SpeakTrigger.Response()

        if (request.status == 0): # 0 
            self.bb_speak(str(request.message))
        elif (request.status == 1):
            # self.bb_speak("Hello, I am speaking!")
            self.bb_speak("Sheehhhhhhhhhhhhhhhhhhhhhhhhhhhhhh")


        response.success = True
        return response
    
    
def main(args=None):
    rclpy.init(args=args)
    node = brobot_speak()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

