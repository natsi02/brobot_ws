#!/usr/bin/python3
from tensorflow.keras.models import load_model
import cv2
import tensorflow as tf
import numpy as np
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge
import torch
from facenet_pytorch import InceptionResnetV1, MTCNN
from tqdm import tqdm
from types import MethodType
import os

class brobot_face_detection(Node):
    def __init__(self):
        super().__init__('face_node')

        ### load model
        self.resnet = InceptionResnetV1(pretrained='vggface2').eval()
        self.mtcnn = MTCNN(image_size=224, keep_all=True, thresholds=[0.4, 0.5, 0.5], min_face_size=60)
        self.mtcnn.detect_box = MethodType(self.detect_box, self.mtcnn)        

        ### get encoded features for all saved images
        self.saved_pictures = "/home/natsi02/brobot_ws/src/brobot_controller/scripts/saved"
        self.all_people_faces = {}

        for file in os.listdir(self.saved_pictures):
            person_face, extension = file.split(".")
            img = cv2.imread(f'{self.saved_pictures}/{person_face}.jpg')
            cropped = self.mtcnn(img)
            if cropped is not None:
                self.all_people_faces[person_face] = self.encode(cropped)[0, :]

        self.predict_face = None
        self.user_camera_detect = self.create_publisher(Bool, "/camera_detect", 10)
        self.image_sub = self.create_subscription(Image, '/video_frames', self.image_callback,10)
        self.create_timer(0.5, self.timer_callback)
        self.br = CvBridge()
        self.user_name = None
        self.user_get_name = self.create_publisher(String, "/username", 10)

    def detect_box(self, img, save_path=None):
        # Detect faces
        batch_boxes, batch_probs, batch_points = self.bb_face_detection(img, landmarks=True)
        # Select faces
        if not self.keep_all:
            batch_boxes, batch_probs, batch_points = self.select_boxes(
                batch_boxes, batch_probs, batch_points, img, method=self.selection_method
            )
        # Extract faces
        faces = self.extract(img, batch_boxes, save_path)
        return batch_boxes, faces
    
    ### helper function
    def encode(self,img):
        res = self.resnet(torch.Tensor(img))
        return res   

    def timer_callback(self):
        message = Bool()
        message_name = String()
        message_name.data = str(self.user_name)
        if (self.predict_face):
            message.data = True
            self.user_get_name.publish(message_name)
            self.user_name = None   
        else:
            message.data = False
        self.user_camera_detect.publish(message)

    def image_callback(self, msg:Image):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        self.bb_face_detection(current_frame)
        cv2.imshow("face detection", current_frame)
        cv2.waitKey(1)

    def bb_face_detection(self, frame) -> None:
        
        batch_boxes, cropped_images = self.mtcnn.detect_box(frame)

        if cropped_images is not None:
            for box, cropped in zip(batch_boxes, cropped_images):
                x, y, x2, y2 = [int(x) for x in box]
                img_embedding = self.encode(cropped.unsqueeze(0))
                detect_dict = {}
                for k, v in self.all_people_faces.items():
                    detect_dict[k] = (v - img_embedding).norm().item()
                min_key = min(detect_dict, key=detect_dict.get)

                self.predict_face = True
                self.user_name = min_key

                # if min_key is not None:
                #     print(f"We detected {min_key} the value is {detect_dict[min_key]}")
                if detect_dict[min_key] >= 0.7: #Threshold 0.7
                    min_key = 'Undetected'
                    self.predict_face = False            
                    
                cv2.rectangle(frame, (x, y), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, min_key, (x + 5, y + 10), 
                cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
                    
        ### display
        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()
    # # -> search verbose to delete predict
    # def bb_face_detection(self, frame) -> None:
    #     frame = frame[50:500, 50:500,:]
        
    #     rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    #     yhat = self.facetracker.predict(np.expand_dims(tf.image.resize(rgb, (120,120))/255,0))
    #     sample_coords = yhat[1][0]

    #     if yhat[0] > 0.5:
    #         self.predict_face = True
    #         self.user_name = "Beep" # From Label


    #         # Controls the main rectangle
    #         cv2.rectangle(frame, 
    #                     tuple(np.multiply(sample_coords[:2], [450,450]).astype(int)),
    #                     tuple(np.multiply(sample_coords[2:], [450,450]).astype(int)), 
    #                             (255,0,0), 2)
    #         # Controls the label rectangle
    #         cv2.rectangle(frame, 
    #                     tuple(np.add(np.multiply(sample_coords[:2], [450,450]).astype(int), 
    #                                     [0,-30])),
    #                     tuple(np.add(np.multiply(sample_coords[:2], [450,450]).astype(int),
    #                                     [80,0])), 
    #                             (255,0,0), -1)
            
    #         # Controls the text rendered
    #         cv2.putText(frame, 'face', tuple(np.add(np.multiply(sample_coords[:2], [450,450]).astype(int),
    #                                             [0,-5])),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

            
            
    #     else:
    #         self.predict_face = False
        
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = brobot_face_detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
