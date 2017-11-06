#!/usr/bin/env python

import rospy
import rospkg
import math
import os.path
import numpy as np
import errno
import cv2
from opt_msgs.msg import StandardSkeletonTrackArray

valid_skeletons = 0
skeletons = []
median = np.zeros(shape=(3, 15))
end_node = False
pose_name = ""

def generatestickman(points):
  stickman = np.zeros((300, 300, 1), dtype = "uint8")
  cv2.line(stickman, (int(points[:,0][0]),int(points[:,0][1])), (int(points[:,1][0]), int(points[:,1][1])), 255, 10)
  cv2.line(stickman, (int(points[:,1][0]),int(points[:,1][1])), (int(points[:,2][0]), int(points[:,2][1])), 255, 10)
  cv2.line(stickman, (int(points[:,2][0]),int(points[:,2][1])), (int(points[:,3][0]), int(points[:,3][1])), 255, 10)
  cv2.line(stickman, (int(points[:,3][0]),int(points[:,3][1])), (int(points[:,4][0]), int(points[:,4][1])), 255, 10)
  cv2.line(stickman, (int(points[:,1][0]),int(points[:,1][1])), (int(points[:,5][0]), int(points[:,5][1])), 255, 10)
  cv2.line(stickman, (int(points[:,5][0]),int(points[:,5][1])), (int(points[:,6][0]), int(points[:,6][1])), 255, 10)
  cv2.line(stickman, (int(points[:,6][0]),int(points[:,6][1])), (int(points[:,7][0]), int(points[:,7][1])), 255, 10)
  cv2.line(stickman, (int(points[:,8][0]),int(points[:,8][1])), (int(points[:,9][0]), int(points[:,9][1])), 255, 10)
  cv2.line(stickman, (int(points[:,9][0]),int(points[:,9][1])), (int(points[:,10][0]),int(points[:,10][1])), 255, 10)
  cv2.line(stickman, (int(points[:,11][0]),int(points[:,11][1])), (int(points[:,12][0]), int(points[:,12][1])), 255, 10)
  cv2.line(stickman, (int(points[:,12][0]),int(points[:,12][1])), (int(points[:,13][0]), int(points[:,13][1])), 255, 10)
  cv2.line(stickman, (int(points[:,2][0]),int(points[:,2][1])), (int(points[:,8][0]), int(points[:,8][1])), 255, 10)
  cv2.line(stickman, (int(points[:,8][0]),int(points[:,8][1])), (int(points[:,11][0]), int(points[:,11][1])), 255, 10)
  cv2.line(stickman, (int(points[:,11][0]),int(points[:,11][1])), (int(points[:,5][0]), int(points[:,5][1])), 255, 10)
  # cv2.imshow("stickman",stickman)
  stickman = cv2.flip(stickman, 1)
  return stickman

def record_frame():
    global end_node
    global pose_name
    global median
    # read CSV file
    rospack = rospkg.RosPack()
    gallery_dir = rospack.get_path('gallery_poses') + "/data"
    csv_filename = gallery_dir + "/poses.csv"
    frame_id = 0
    frame_dir = gallery_dir
    # CSV file updating, frame_folder updating, frame_id updating
    if not os.path.isfile(csv_filename):
        csv_file = open(gallery_dir + "/poses.csv", "w")
        csv_file.write("1\n")
        csv_file.write("0,1,"+pose_name.lower()+"\n")
        csv_file.close()
        frame_dir = gallery_dir + "/0"
        if not os.path.isdir(frame_dir):
            try:
                os.makedirs(frame_dir)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
                else:
                    os.rmdir(frame_dir)
                    os.makedirs(frame_dir)
    else:
        csv_file = open(gallery_dir + "/poses.csv", "r+")
        content = csv_file.readlines()
        content = [line.strip().split(",") for line in content]
        # check for pose_name existance
        pose_found = False
        line_id = 1
        for line in content[1:]:
            if pose_name in line:
                pose_found = True 
                break
            line_id += 1
        if pose_found:  
            frame_id = int(content[line_id][1])
            frame_dir = gallery_dir + "/" + content[line_id][0]
            try:
                os.makedirs(frame_dir)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
            # read number of frames in the correct folder
            content[line_id][1] = str(frame_id + 1)
            content = [",".join(line) + "\n" for line in content]
            # read num_frame and register a new one with num_frame as ID
            # update content by writing the new num_frame in the correct row
        else:
            print (content)
            content[0][0] = str(int(content[0][0]) + 1) # new poses
            content = [",".join(line) + "\n" for line in content]
            content.append(str(len(content) - 1) + ",1," + pose_name + "\n")
            pose_id = len(content) - 2
            frame_dir = gallery_dir + "/" + str(pose_id)
            if not os.path.isdir(frame_dir):
                try:
                    os.makedirs(frame_dir)
                except OSError as e:
                    if e.errno != errno.EEXIST:
                        raise  # raises the error again
        csv_file.close()
        csv_file = open(gallery_dir + "/poses.csv", "w")
        csv_file.writelines(content)
        csv_file.close()
    # write the new frame_id (and the stickman)
    frame_file = open(frame_dir + "/frame_" + str(frame_id) + ".txt", "w")
    for r in xrange(median.shape[0]):
        line = str(median[r,0])
        for c in xrange(1, median.shape[1]):
            line += " " + str(median[r,c])
        line += "\n"
        frame_file.write(line)
    frame_file.close()
    stickman = np.zeros((300, 300, 1), dtype = "uint8")
    old_min = np.min(median, axis=1)
    old_max = np.max(median, axis=1)
    old_diff = old_max - old_min
    new_min = np.array([50,50,50])
    new_max = np.array([250,250,250])
    new_diff = new_max - new_min
    # cv2.line(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) 
    # frame
    # stickman
    points = ((median.transpose() - old_min) / old_diff * new_diff + new_min).transpose()
    points_old = points
    matr_rot_z = np.matrix("0 -1 0; 1 0 0; 0 0 1")
    matr_rot_x = np.matrix("0 0 1; 0 1 0; -1 0 0")
    points = (points.transpose() * matr_rot_x * np.linalg.inv(matr_rot_z) ).transpose()
    points -= (points[:,14].transpose() - (150,150,0)).transpose()
    # points = ((points - old_min) / old_diff * new_diff + new_min).transpose()
    stickman = generatestickman(points)
    cv2.imwrite(frame_dir + "/frame_" + str(frame_id) + "_front.jpg", stickman)
    stickman = generatestickman(points_old)
    cv2.imwrite(frame_dir + "/frame_" + str(frame_id) + "_top.jpg", stickman)
    print ("Recorded!")
    end_node = True

def reset():
    global valid_skeletons
    global skeletons
    global median
    soft_reset()
    valid_skeletons = 0
    
def soft_reset():
    global valid_skeletons
    global skeletons
    global median
    skeletons = []
    median = np.zeros(shape=(3, 15))

def callback(data):
    global valid_skeletons
    global skeletons
    global median
    # Selecting the closest skeleton to /world
    cur_skel = -1
    distance = 200000
    for i, skel in enumerate(data.tracks):
        if distance > skel.distance:
            cur_skel = i
    if cur_skel == -1:
        return
    skeleton = data.tracks[cur_skel]
    skeleton_matrix = np.zeros(shape=(3, len(skeleton.joints)))
    for i, joint in enumerate(skeleton.joints):
        skeleton_matrix[:, i] = [joint.x, joint.y, joint.z]
    # check validity
    if np.isnan(np.sum(skeleton_matrix[:,joints_to_consider_by_index])):
        print ("NaNs!")
        reset()
        return
    if np.all(np.abs(skeleton_matrix) < 0.03):
        print ("all zeros!")
        reset()
        return
    valid_skeletons += 1
    # check pose
    norms = np.zeros(len(skeleton.joints))
    if not len(skeletons) == 0:
        for i in xrange(skeleton_matrix.shape[1]):
            if i not in joints_to_consider_by_index: continue
            norms[i] = np.linalg.norm(median[:,i] - skeleton_matrix[:,i])
        if np.any(norms > per_joint_valid_pose_threshold):
            print ("Not valid pose!")
            soft_reset()
            return
    skeletons.append(skeleton_matrix)
    print ("Validpose:" + str(len(skeletons)) + " / " + str(min_valid_poses))
    median = np.median(np.array(skeletons), axis=0)
    # check record
    if len(skeletons) > min_valid_poses:
        record_frame()
        reset()
        return
    
if __name__ == "__main__":
    rospy.init_node("pose_recorder")
    rospy.Subscriber("skeletons", StandardSkeletonTrackArray, callback, queue_size=10)
    min_valid_skeletons = rospy.get_param("~min_valid_skeletons", 100)
    print ("min_valid_skeletons: " + str(min_valid_skeletons))
    per_joint_valid_pose_threshold = rospy.get_param("~per_joint_valid_pose_threshold", 0.01)
    print ("per_joint_valid_pose_threshold: " + str(per_joint_valid_pose_threshold))
    min_valid_poses = rospy.get_param("~min_valid_poses", 100)
    print ("min_valid_poses" + str(min_valid_poses))
    joints_to_consider_by_index = np.array(rospy.get_param("~joints_to_consider_by_index", [0,1,2,3,4,5,6,7,8,9,11,12,13,14]))
    print (joints_to_consider_by_index)
    if not rospy.has_param('~pose_name'):
        print ("ERROR: pose_name not set!")
        import sys
        sys.exit(1)
    pose_name = rospy.get_param("~pose_name").lower()
    while True:
        rospy.sleep(0.001)
        if rospy.is_shutdown() or end_node:
            import sys
            sys.exit(0)
