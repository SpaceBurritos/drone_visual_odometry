import airsim
import threading
import assistant
import queue
import numpy as np

class Clients:
    def __init__(self):
        self.stop_thread = False
        self.dir = "./img"
        self.queue = queue.Queue()
        self.client_drone = airsim.MultirotorClient()
        self.client_measures = airsim.MultirotorClient()
        self.client_drone.confirmConnection()
        self.client_measures.confirmConnection()
        self.client_drone.enableApiControl(True)
        self.client_drone.armDisarm(True)
        pass

    def end(self):
        self.client_drone.armDisarm(False)
        self.client_drone.enableApiControl(False)
        self.client_drone.reset()
        self.client_measures.reset()
        pass

    def move_drone(self, client):
        client.moveToPositionAsync(-10, 0, 0, 3).join()
        client.moveToPositionAsync(-10, 10, 0, 3).join()
        client.moveToPositionAsync(0, 10, 0, 3).join()
        client.moveToPositionAsync(0, 0, 0, 3).join()

    def take_photos(self, client, tmp_dir="./img"):
        self.dir = tmp_dir
        counter = 0
        while not self.stop_thread:
            self.queue.put(assistant.request_images(client, counter, self.dir))
            counter += 1

    def move_and_take_photos(self, tmp_dir="./img"):
        self.dir = tmp_dir
        x = threading.Thread(target=self.move_drone, args=(self.client_drone,))
        y = threading.Thread(target=self.take_photos, args=(self.client_measures, ))
        x.start()
        y.start()
        x.join()
        self.stop_thread = True
        y.join()

    def takeoff(self):
        self.client_drone.takeoffAsync().join()


        pass