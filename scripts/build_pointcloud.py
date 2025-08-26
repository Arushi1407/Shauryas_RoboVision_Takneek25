#!/usr/bin/env python3
import cv2
import numpy as np
import os
import open3d as o3d

# -----------------------------
# 1. Load images
# -----------------------------
image_folder = "../recon_images"  # path from scripts folder
images = [cv2.imread(os.path.join(image_folder, f)) for f in sorted(os.listdir(image_folder))]

# -----------------------------
# 2. Feature Extraction (ORB)
# -----------------------------
orb = cv2.ORB_create(nfeatures=2000)
keypoints_list, descriptors_list = [], []

for img in images:
    kp, des = orb.detectAndCompute(img, None)
    keypoints_list.append(kp)
    descriptors_list.append(des)

# -----------------------------
# 3. Feature Matching
# -----------------------------
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches_all = []

for i in range(len(images)-1):
    matches = bf.match(descriptors_list[i], descriptors_list[i+1])
    matches = sorted(matches, key=lambda x: x.distance)
    matches_all.append(matches)

# -----------------------------
# 4. Triangulation (first two images)
# -----------------------------
fx, fy = 525.0, 525.0
cx, cy = 400.0, 400.0
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])

pts1 = np.float32([keypoints_list[0][m.queryIdx].pt for m in matches_all[0]]).T
pts2 = np.float32([keypoints_list[1][m.trainIdx].pt for m in matches_all[0]]).T

P1 = K @ np.hstack((np.eye(3), np.zeros((3,1))))
R = np.eye(3)
t = np.array([[0.1],[0],[0]])
P2 = K @ np.hstack((R, t))

pts_4d = cv2.triangulatePoints(P1, P2, pts1[:2], pts2[:2])
pts_3d = (pts_4d[:3] / pts_4d[3]).T

# -----------------------------
# 5. Assign Colors
# -----------------------------
colors = []
img = images[0]
for pt in pts1.T:
    x, y = int(pt[0]), int(pt[1])
    colors.append(img[y, x]/255.0)
colors = np.array(colors)

# -----------------------------
# 6. Build Open3D Point Cloud
# -----------------------------
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts_3d)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])

