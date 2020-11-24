'''
    Takes as input a csv file in TUM format and outputs a csv file in TUM format
    Noisifies the poses from the input file with some random noise 
    Noise parameters are user inputs
'''

import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R 


def addNoiseToPose(pose, SIGMA_XYZ, SIGMA_ROT):
    nPose = pose.copy()
    tr_err = np.random.normal(0, SIGMA_XYZ, 3)  # translational noise
    rot_err = np.random.normal(0, SIGMA_ROT, 3) #noise on euler angles
    nPose[:3, 3] += tr_err
    corrected_euler = R.from_matrix(nPose[:3, :3]).as_euler('zyx', degrees=True) + rot_err
    nPose[:3, :3] = R.from_euler('zyx', corrected_euler, degrees=True).as_matrix()

    return nPose

def main(inputFile, outputFile, noise_xyz, noise_rot):
    if not inputFile.endswith(".csv") or not outputFile.endswith(".csv"):
        print("Invalid file format, expecting .csv")
        return

    with open(inputFile, 'r') as f:
        with open(outputFile, 'w') as g:
            allLinesF = list(map(str.strip, f.readlines()))
            for lineF in allLinesF:
                tr = list(map(float, lineF.split()[1:4]))
                rotQ = list(map(float, lineF.split()[4:]))
                rotM = R.from_quat(rotQ).as_matrix()
                pose = np.eye(4)
                pose[:3,:3] = rotM
                pose[:3, 3] = tr
                nPose = addNoiseToPose(pose, noise_xyz, noise_rot)
                print(pose)
                print(nPose)
                nPoseRotQ = list(R.from_matrix(nPose[:3, :3]).as_quat())
                g.write(lineF.split()[0]+" "+" ".join(list(map(str,nPose[:3, 3])))+ " " +" ".join(list(map(str,nPoseRotQ)))+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("noise_xyz")
    parser.add_argument("noise_rot")
    res = parser.parse_args()
    main(res.input, res.output, float(res.noise_xyz), float(res.noise_rot))
