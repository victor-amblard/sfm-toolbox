'''
    Script to undistort images
    Takes a directory as argument and saves in the same directory undistorted
'''
import os
import cv2
import argparse
import numpy as np
from tqdm import tqdm
import multiprocessing as mp

IMAGE_EXTENSION = ".jpg"
SUFFIX = "_rectified"


def undistort(imgFn):
    img = cv2.imread(imgFn)
    cameraMatrix  = np.array([ [1759.963229, 0.000000, 1000.649809], 
                    [0.000000, 1760.305409, 764.933005], 
                    [0, 0, 1]])

    distsCoeff = np.array([0.136323, 0.684112, 0.001126, -0.004875, 0.000000])
    
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix,distsCoeff,(w,h),1,(w,h))
    uImg = cv2.undistort(img, cameraMatrix, distsCoeff, None, newcameramtx)
    x,y,w,h = roi
    uImg = uImg[y:+y+h, x:x+w]
    
    cv2.imwrite(os.path.join(imgFn.split(IMAGE_EXTENSION)[0]+SUFFIX+IMAGE_EXTENSION), uImg)

def undistortAllImages(allImages):
    validFilenames = [os.path.join(wd, fn) for fn in allImages if fn.endswith(IMAGE_EXTENSION)]
    
    with mp.Pool(processes=4) as p:
        with tqdm(total=len(validFilenames)) as pbar:
            for i, _ in enumerate(p.imap_unordered(undistort, validFilenames)):
                pbar.update()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('wd')
    args = parser.parse_args()

    wd = args.wd
    allImages = os.listdir(wd)
    undistortAllImages(allImages)