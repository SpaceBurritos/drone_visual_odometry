# import setup_path
import airsim
from slam import Slam
import assistant
from clients import Clients
import numpy as np


tmp_dir = "./img"
assistant.create_directory(tmp_dir)

clients = Clients()
slam = Slam()

airsim.wait_key('Press any key to takeoff')

slam.calibrate_camera(clients.client_2)
clients.takeoff()

assistant.print_state(clients.client_1)

clients.move_and_take_photos()
slam.append(np.array(clients.queue.queue))
#print(slam.images)

#print(slam.images.shape)
slam.estimate_trajectory_dataset()
slam.plot_trajectory()


clients.end()
