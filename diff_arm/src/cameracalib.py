#!/usr/bin/env python

"""
https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/cameracalib.py
https://www.youtube.com/watch?v=QV1a1G4IL3U

Usage:
cameracalib.py  <folder> <image type> <num rows> <num cols> <cell dimension>

--h for help

"""

# import necessary packages
from __future__ import print_function
import roslib
# roslib.load_manifest('object_track')
import rospy
import numpy as np
import cv2
import glob
import sys
import argparse


def main(): 

    # -----set default parameters---------
    nRows = 6
    nCols = 8
    dimension = 27 #- mm
    workingFolder = "/home/nardos/Pictures/object_track_snaps"
    imageType = 'jpg'
    # ------------------------------------

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((nRows*nCols,3), np.float32)
    objp[:,:2] = np.mgrid[0:nCols,0:nRows].T.reshape(-1,2) * dimension
    print(objp)

    # arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # if args is less than 6 then will use default values
    # if len(sys.argv) < 6:
    #         print("\n Not enough inputs are provided. Using the default values.\n\n" \
    #               " type -h for help")
    # else:
    #     workingFolder   = sys.argv[1]
    #     imageType       = sys.argv[2]
    #     nRows           = int(sys.argv[3])
    #     nCols           = int(sys.argv[4])
    #     dimension       = float(sys.argv[5])

    # print usage state if -h or --h is parsed
    if '-h' in sys.argv or '--h' in sys.argv:
        print("\n IMAGE CALIBRATION GIVEN A SET OF IMAGES")
        print(" call: python cameracalib.py <folder> <image type> <num rows (9)> <num cols (6)> <cell dimension (25)>")
        print("\n The script will look for every image in the provided folder and will show the pattern found." \
              " User can skip the image pressing ESC or accepting the image with RETURN. " \
              " At the end the end the following files are created:" \
              "  - cameraDistortion.txt" \
              "  - cameraMatrix.txt \n\n")

        sys.exit()

    # Find the images files
    filename = workingFolder + "/*." + imageType
    images = glob.glob(filename)

    # print the amount of images to process
    print(len(images))

    # need 9 images or more to calibrate successfully
    if len(images) < 9:
        print("Not enough images were found: at least 9 shall be provided!!!")
        sys.exit()

    else:
        nPatternFound = 0
        imgNotGood = images[1]

        for fname in images:
            if 'calibresult' in fname: continue
            
            # read the file and convert in grayscale
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            print("Reading image ", fname)

            # find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (nCols,nRows),None)

            # if found, add object points, image points (after refining them)
            if ret == True:
                print("Pattern found! Press ESC to skip or ENTER to accept")
                
                # refine the picture to make better
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, (nCols,nRows), corners2, ret)
                cv2.imshow('img',img)
                
                # wait and if esc button pressed then image is skipped
                k = cv2.waitKey(0) & 0xFF
                if k == 27: # ESC Button
                    print("Image Skipped")
                    imgNotGood = fname
                    continue

                # enter button pressed, image accepted, add object/image points
                print("Image accepted")
                nPatternFound += 1
                objpoints.append(objp)
                imgpoints.append(corners2)

            else:
                imgNotGood = fname

    cv2.destroyAllWindows()

    if (nPatternFound > 1):
        # show good images accepted
        print("Found %d good images" % (nPatternFound))

        # calibrate camera based on 2D and 3D points
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        # undistort any poor images
        img = cv2.imread(imgNotGood)
        h,  w = img.shape[:2]
        print("Image to undistort: ", imgNotGood)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

        # undistort by remapping
        mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
        dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        print("ROI: ", x, y, w, h)

        # save the image, print camera matrix, distortion, rotation, and translation vectors.
        cv2.imwrite(workingFolder + "/calibresult.png",dst)
        print("Calibrated picture saved as calibresult.png")
        print("Calibration Matrix: ")
        print(mtx)
        print("Distortion: ", dist)
        print("Rotation Vetor: ", cv2.Rodrigues(rvecs[-2]))
        print("Translation Vector: ", tvecs)

        # save results to text file
        filename = workingFolder + "/cameraMatrix.txt"
        np.savetxt(filename, mtx, delimiter=',')
        filename = workingFolder + "/cameraDistortion.txt"
        np.savetxt(filename, dist, delimiter=',')
        # filename = workingFolder + "/translationVector.txt"
        # np.savetxt(filename, tvecs, delimiter=',')

        # calculate the error with the results
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error

        print ("total error: ", mean_error/len(objpoints))

    else:
        print ("In order to calibrate you need at least 9 good pictures... try again")

if __name__ == "__main__":
    main()





