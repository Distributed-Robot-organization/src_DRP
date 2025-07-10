#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
import time
from datetime import datetime
import select
import sys
import termios
import tty

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.bridge = CvBridge()
        self.current_image = None
        self.capturing = False
        self.save_dir = os.path.join("..", "data_images")
        
        # Crea directory e subscriber
        os.makedirs(self.save_dir, exist_ok=True)
        self.create_subscription(Image, 'scanner/image_raw', self.image_callback, 10)
        
        print("Script avviato! Premi 'B' per acquisire 10 immagini, 'Q' per uscire")
        
    def image_callback(self, msg):
        """Callback per ricevere le immagini dal topic ROS"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"Errore conversione immagine: {e}")
    
    def capture_sequence(self):
        """Acquisisce una sequenza di 10 immagini, una ogni secondo"""
        if self.capturing:
            return print("Acquisizione già in corso...")
        
        if self.current_image is None:
            return print("Nessuna immagine disponibile. Verifica che la fotocamera sia attiva.")
        
        self.capturing = True
        print("Acquisizione di 10 immagini in corso...")
        session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        for i in range(10):
            if self.current_image is not None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"image_{session_timestamp}_{i+1:02d}_{timestamp}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                cv2.imwrite(filepath, self.current_image)
                print(f"Salvata {i+1}/10: {filename}")
            
            if i < 9:
                time.sleep(1)
        
        print("Acquisizione completata!")
        self.capturing = False
    
    @staticmethod
    def get_char():
        """Legge un singolo carattere da terminale"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    image_capture = ImageCapture()
    
    def keyboard_handler():
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = image_capture.get_char().lower()
                    if char == 'b':
                        threading.Thread(target=image_capture.capture_sequence, daemon=True).start()
                    elif char == 'q':
                        rclpy.shutdown()
                        return
                time.sleep(0.1)
        except KeyboardInterrupt:
            rclpy.shutdown()
    
    threading.Thread(target=keyboard_handler, daemon=True).start()
    
    try:
        rclpy.spin(image_capture)
    except Exception as e:
        print(f"Errore: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()