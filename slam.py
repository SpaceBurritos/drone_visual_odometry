import cv2
import numpy as np
import assistant
import matplotlib.pyplot as plt


class Slam:

    def __init__(self):
        self.matches = []
        self.kp_list = []
        self.des_list = []
        self.trajectory = [[0, 0, 0]]
        self.k = None
        self.images = []
        self.cx = 0
        self.cy = 0
        self.focal_length = 0

    def append(self, images):
        self.images = images

    def extract_feature_cont(self, img):
        orb = cv2.ORB_create(nfeatures=200, scaleFactor=2, edgeThreshold=10, scoreType=cv2.ORB_HARRIS_SCORE, fastThreshold=10)
        kp, des = orb.detectAndCompute(img, None)
        return kp, des

    def extract_feature_dataset(self):
        for img in self.images:
            kp, des = self.extract_feature_cont(img)
            self.kp_list.append(kp)
            self.des_list.append(des)

    def feature_matching_cont(self, des1, des2):
        #print("entered feature_matching_cont")
        FLANN_INDEX_LSH = 6

        index_params = dict(algorithm=FLANN_INDEX_LSH,
                            table_number=6,  # 12
                            key_size=12,  # 20
                            multi_probe_level=1)  # 2
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        match = flann.knnMatch(des1, des2, k=2)
        ratio_thresh = 0.7
        good_matches = []
        for i, pair in enumerate(match):
            try:
                m, n = pair
                if m.distance < ratio_thresh * n.distance:
                    good_matches.append(m)
            except ValueError:
                pass
        return good_matches

    def feature_matching_dataset(self):
        print("entered feature_matching_dataset")
        for i in range(len(self.des_list) - 1):
            print(i)
            des1 = self.des_list[i]
            des2 = self.des_list[i + 1]
            match = self.feature_matching_cont(des1, des2)
            self.matches.append(match)

    def estimate_trajectory_cont(self, match, kp1, kp2, depth=None):

        image1_points = []
        image2_points = []
        for m in match:
            query_idx = m.queryIdx
            train_idx = m.trainIdx
            p1_x, p1_y = kp1[query_idx].pt
            image1_points.append([p1_x, p1_y])

            p2_x, p2_y = kp2[train_idx].pt
            image2_points.append([p2_x, p2_y])
        print(image1_points)
        print(image2_points)
        E, mask = cv2.findEssentialMat(np.array(image1_points), np.array(image2_points), self.k)
        print(E)
        retval, rmat, tvec, mask = cv2.recoverPose(E, np.array(image1_points), np.array(image2_points), self.k)

        return rmat, tvec, image1_points, image2_points

    def estimate_trajectory_dataset(self, depth_maps=[]):
        print("entered")
        self.extract_feature_dataset()
        self.feature_matching_dataset()
        for i in range(len(self.matches)):
            match = self.matches[i]
            kp1 = self.kp_list[i]
            kp2 = self.kp_list[i + 1]

            rmat, tvec, image1_points, image2_points = self.estimate_trajectory_cont(match, kp1, kp2)
            R = rmat
            t = np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0]])

            P_new = np.eye(4)
            P_new[0:3, 0:3] = R.T
            P_new[0:3, 3] = (-R.T).dot(t)
            new_trajectory = P_new[:3, 3]
            self.trajectory.append(new_trajectory)
        self.trajectory = np.array(self.trajectory).T
        self.trajectory[2,:] = -self.trajectory[2,:]

    def calibrate_camera(self, client):
        img = assistant.request_images(client, 0, "./img")
        width = img.shape[1]
        height = img.shape[0]
        focal_length = cx = width / 2
        cy = height / 2
        s = 0
        horizontal_fov = 2 * np.arctan2(width, 2 * focal_length)
        self.k = np.array([[focal_length, 0, 0],
                           [s, focal_length, 0],
                           [cx, cy, 1]])

    def plot_trajectory(self):
        trajectories = np.cumsum(self.trajectory, axis=1)
        x = trajectories[0]
        y = trajectories[1]
        z = trajectories[2]
        plt.plot(x, y)
        plt.plot(x, z)
        plt.plot(y, z)
        plt.show()
