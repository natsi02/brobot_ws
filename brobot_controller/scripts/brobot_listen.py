#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int8
from brobot_interfaces.srv import ListenTrigger
class brobot_listen(Node):
    def __init__(self):
        super().__init__('listen_node')
        self.recognizer = sr.Recognizer()
        self.listen_status = None 
        self.text = None
        self.trigger_listen = self.create_service(ListenTrigger, '/trigger_listen', self.trigger_listen)


    def bb_listen(self) -> None:
        with sr.Microphone() as source:
            print("Say something:")
            audio = self.recognizer.listen(source, timeout=1)
        try:
            # Use Google Web Speech API to recognize the audio
            self.text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Get Speech Data: {self.text} ' ) 
            self.listen_status = 0

        except sr.UnknownValueError:
            self.get_logger().info(f"Google Web Speech API could not understand audio" ) 
            self.listen_status = 1

        except sr.RequestError as e:
            self.get_logger().info(f"Could not request results from Google Web Speech API; {e}" ) 
            self.listen_status = 2


    def trigger_listen(self, request, response):
        self.bb_listen()
        response = ListenTrigger.Response()
        response.status = self.listen_status
        response.message = str(self.text)
        print(response.message)
        return response

    


def main(args=None):
    rclpy.init(args=args)
    node = brobot_listen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
