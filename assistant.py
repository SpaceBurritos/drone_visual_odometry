import pprint
import os
import numpy as np
import airsim
import cv2
import sys


def print_data(client):
    state = client.getMultirotorState()
    s = pprint.pformat(state)
    print("state: %s" % s)

    imu_data = client.getImuData()
    s = pprint.pformat(imu_data)
    print("imu_data: %s" % s)

    barometer_data = client.getBarometerData()
    s = pprint.pformat(barometer_data)
    print("barometer_data: %s" % s)

    magnetometer_data = client.getMagnetometerData()
    s = pprint.pformat(magnetometer_data)
    print("magnetometer_data: %s" % s)

    gps_data = client.getGpsData()
    s = pprint.pformat(gps_data)
    print("gps_data: %s" % s)


def print_state(client):
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))

'''
def save_images(responses, tmp_dir, name):
    for idx, response in enumerate(responses):

        filename = os.path.join(tmp_dir, str(name))

        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress:  # png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else:  # uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)  # get numpy array
            img_rgb = img1d.reshape(response.height, response.width,
                                    3)  # reshape array to 4 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb)  # write to png
'''


def request_images(client, name, tmp_dir, save_image = False):
    rawImage, = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    if (rawImage == None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    elif save_image:
        pass#save_images(rawImage, tmp_dir, name)
    image = np.frombuffer(rawImage.image_data_uint8, dtype=np.uint8).reshape(rawImage.height, rawImage.width, 3)
    image = np.flipud(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image

def create_directory(tmp_dir):
    try:
        os.makedirs(tmp_dir)
    except OSError:
        if not os.path.isdir(tmp_dir):
            raise
