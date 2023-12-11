#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import speech_recognition as sr
from rclpy.action import ActionServer
from brobot_interfaces.action import ListenAction

class brobot_listen(Node):
    def __init__(self):
        super().__init__('listen_action_server')
        self.recognizer = sr.Recognizer()
        self.listen_status = None 
        self.text = None
        self._action_server = ActionServer(self,
            ListenAction,
            '/listen_action',
            self.execute_callback)

    def bb_listen(self) -> None:
        with sr.Microphone() as source:
            print("Say something:")
            audio = self.recognizer.listen(source, timeout=2)
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

    def execute_callback(self, listen_handle):        
        self.bb_listen()
        listen_handle.succeed()
        result = ListenAction.Result()
        result.message = str(self.text)
        result.status = self.listen_status
        return result



def main(args=None):
    rclpy.init(args=args)
    node = brobot_listen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
