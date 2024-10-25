import cv2
from threading import Thread
import json
import numpy as np


class RemoteCam:
    def __init__(self, ip, port=8005, name="IPCAM"):
        '''
        `.cap_tread.start()` to start the thread
        `.last_frame` to get the latest frame
        `.kill_cam()` to kill the thread
        '''
        self.ip = ip
        self.port = port
        self.name = name

        self.kill = False

        print(f"Attempting to connect to {self.ip}:{self.port}")
        self.cap = cv2.VideoCapture(f"tcp://{self.ip}:{self.port}")
        self.last_frame = None
        self.cap_thread = Thread(target=self.update, args=())

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print(f"Error: failed to capture image from {self.ip}!")
            return None
        return frame

    def release(self):
        self.cap.release()

    def __del__(self):
        self.release()


    def read_intrinsics(self) -> None:
        with open(f"calibration_data/{self.name}.json", 'r') as f:
            d = json.load(f)

        self.k, self.dist = np.array(d['mtx']), np.array(d['dist'])

    def kill_cam(self):
        self.kill = True

    # THREAD ---
    def update(self):
      print(f"Starting thread for {self.ip}")
      while not self.kill:
        self.last_frame = self.get_frame()
        if self.last_frame is None:
          break