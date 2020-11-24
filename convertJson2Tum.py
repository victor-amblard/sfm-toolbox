'''
    Converts poses from an openMVG JSON output file to a CSV Tum-like file
    Usage: python3 sfm_data.json output.csv [scaling_parameters.yaml]
    @author Victor Amblard
'''

import json
import yaml
import rospy
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R 


def removePrefixSuffix(fn):
    s, ns = list(map(int, fn.split(".")[0].split("_")[1:3]))
    return rospy.Time(s, ns)

def convertPoses(sfmOutput, oFilename, transform = np.eye(4), scale = 1):
    print(transform)
    transform[:3,3] /= scale
    with open(sfmOutput, 'r') as f:
        
        data = json.load(f)
        allPoses = data["extrinsics"]
        allViews = data["views"]
        print(str(len(allPoses))+ " poses in JSON file")

        with open(oFilename, 'w') as oFile:
            for i in range(len(allPoses)):
                curData = allPoses[i]["value"]
                x, y, z = curData["center"]
                rot_ = curData["rotation"]
                rot = np.array(rot_).reshape(3,3)
                
                pose_se3 = np.eye(4)
                pose_se3[:3, :3] = np.linalg.inv(rot)
                pose_se3[:3, 3] = np.array([x,y,z])
                
                pose_se3 = transform @ pose_se3 
                pose_se3[:3, 3] *= scale

                qx, qy, qz, qw = R.from_matrix(pose_se3[:3, :3]).as_quat()
                tmstamp = removePrefixSuffix(allViews[i]["value"]["ptr_wrapper"]["data"]["filename"])
                tmstamp = tmstamp - rospy.Duration(0.06)
                nT = str(tmstamp.secs)+"."+str(tmstamp.nsecs).zfill(9)
                oFile.write(" ".join(list(map(str,[nT, pose_se3[0,3],pose_se3[1,3],pose_se3[2,3],qx,qy,qz,qw])))+"\n")

def readYAMLTransformFile(yamFile):
    res = None
    with open(yamFile, 'r') as f:
        res = yaml.load(f, Loader=yaml.FullLoader)
    
    
    return (np.array(res['se3']).reshape(4,4), res['scale'])

if __name__ == "__main__":
     # Parser 
    parser = argparse.ArgumentParser()
    parser.add_argument('input_json_file', type=str, help="OpenMVG's JSON file")
    parser.add_argument('output_csv_file', type=str, help="TUM CSV output file")
    parser.add_argument('yaml_file', type=str, nargs="?", help="Path to YAML file containing the SIM(3) transform")

    resParser = parser.parse_args()
    jsonFile = resParser.input_json_file
    csvFile = resParser.output_csv_file
    
    if not jsonFile.endswith(".json"):
        print("Warning! Input file is not a JSON file!")

    if resParser.yaml_file is not None:
        transform, scale = readYAMLTransformFile(resParser.yaml_file)  # Since openMVG gives an output up to a SIM(3) transform, we might want to change the JSON poses
        convertPoses(jsonFile, csvFile, transform, scale)
    else:
        convertPoses(jsonFile, csvFile)
