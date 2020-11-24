'''
    Reads a YAML file containing all the dataset's info and initalizes openMVG
    Poses must be in a CSV file in TUM format (time, x,y,z,qx,qy,qz)
    @author Victor Amblard
'''

import os
import yaml 
import json
import glob
import rospy
import argparse
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation as R 


IMG_PREFIX = "infra1"
CLOUD_PREFIX  = "cloud"
deltaCameraLidar = rospy.Duration(0.058) # There may be a time difference between LIDAR timestamps and camera timestamps

def getTimeGT(allLinesGT, ptrGT):
    return rospy.Time(*list(map(int, allLinesGT[ptrGT].split()[0].split("."))))

def addPrefixSuffix(rosTime, pref,suf):
    s = str(rosTime.secs)
    ns = str(rosTime.nsecs).zfill(9)
    return pref+"_"+s+"_"+ns+"."+suf

def removePrefixSuffix(fn):
    s, ns = list(map(int, fn.split("/")[-1].split(".")[0].split("_")[1:3]))
    return rospy.Time(s, ns)

def findClosestTimestamp(target, timeList):
    '''
        Binary search to find the closest time 
    '''
    l = 0
    u = len(timeList) - 1
    diff = rospy.Duration(1000)
    val = 0

    while l <= u:
        mid = l + (u-l)//2
        if abs(target - timeList[mid]) < diff:
            val = mid
            diff = abs(target - timeList[mid])
        
        if timeList[mid] < target:
            l = mid + 1
        else:
            u = mid - 1

    return (val, diff)

def associateIMU2Lidar2Images(lidarFolder, imgFolder, csvFile, outputPath, gtFile, threshold = 0.05):
    '''
        Created a .txt file with 3 columns, each row corresponds to a set {pose, image, lidar scan}
        The first row represents the line id in the pose csv file, the second the image filename and the third the scan filename
    '''

    threshold *= 10**9 #max time delta allowed between image timestamp and lidar timestamp (in ns)
    MAX_DELTA = rospy.Duration(0.03) # 30 ms 

    wdLidar = lidarFolder+"*.pcd"
    allTimesLidar = list(map(removePrefixSuffix, glob.glob(wdLidar)))
    allTimesLidar = sorted(allTimesLidar)

    wd = imgFolder+"*.png"
    allTimesCamera = list(map(removePrefixSuffix, glob.glob(wd)))
    allTimesCamera = sorted(allTimesCamera)

    fnOutputCam = os.path.join(os.path.abspath(outputPath), 'filenames.txt')
    allLinesGT = []
    with open(gtFile, 'r') as f:
        allLinesGT = list(map(str.strip, f.readlines()))
    
    ptrGT = 0
    nGT = "selected_gt.csv"
            
    allTimesIMU = []
    allLinesIMU = []
    with open(csvFile, 'r') as f:
        allLinesIMU = list(map(str.strip, f.readlines()))
    allTimesIMU = [x.split()[0] for x in allLinesIMU]
    allTimesIMU = [rospy.Time(int(x.split(".")[0]), int(x.split(".")[1])) for x in allTimesIMU] 
    allTimesIMU = sorted(allTimesIMU)
    count = 0
    with open(os.path.join(outputPath, nGT), 'w') as gt:
        with open(fnOutputCam, 'w') as f:
            for i in range(len(allLinesGT)):
                    s, ns = list(map(int, allLinesGT[i].split()[0].split(".")))
                    curT = rospy.Time(s,ns)
                    correctedCameraTime = curT + deltaCameraLidar
                    
                    imageTimeId, deltaImage = findClosestTimestamp(correctedCameraTime, allTimesCamera)
                    lidarTimeId, deltaLidar = findClosestTimestamp(curT, allTimesLidar)
                    imuFileId, deltaImu = findClosestTimestamp(curT, allTimesIMU)

                    if deltaImage < MAX_DELTA and deltaLidar < MAX_DELTA and deltaImu < MAX_DELTA:
                        lidarTime = allTimesLidar[lidarTimeId]
                        cameraTime = allTimesCamera[imageTimeId]
                        f.write(str(imuFileId)+" "+os.path.join(imgFolder,addPrefixSuffix(cameraTime, IMG_PREFIX, "png"))+" "+os.path.join(lidarFolder,addPrefixSuffix(lidarTime, CLOUD_PREFIX, "pcd"))+"\n")
                        gt.write(allLinesIMU[i]+"\n")
                        count += 1
    
    print("{} locations will be used".format(count))
    return fnOutputCam

def parseYAMLFile(yaml_file):
    '''
        The YAML must contain the following lines/sections
        - imu_file
        - images_folder
        - scans_folder
        - extrinsics
            | imu2camera
            | lidar2camera
        - intrinsics (pinhole model is assumed)
            | fx
            | fy
            | cx
            | cy
    '''
    res = None
    with open(yaml_file, 'r') as f:
        res = yaml.load(f, Loader=yaml.FullLoader)
    return res

def generateInitialJSON(mvgPath, outputPath, yamlData, lstImgFile):     
    lstIntr = ";".join(list(map(str, [yamlData['intrinsics']['fx'],0,yamlData['intrinsics']['cx'],0,yamlData['intrinsics']['fy'], yamlData['intrinsics']['cy'],0,0,1])))
    cmd = ["cd", mvgPath, "&&", "./openMVG_main_SfMInit_ImageListing -i", lstImgFile, "-o",  outputPath, "-c 1",  "-k \"" + lstIntr +"\" -l "+yamlData["scans_folder"]]
    print(" ".join(cmd))
    process = subprocess.Popen(" ".join(cmd), shell=True)
    process.wait()

def loadAllPoses(csvFile, associationFile, update):
    allPoses, validLines = [], []
    
    with open(associationFile, 'r') as f:
        allLines = list(map(str.strip, f.readlines()))
        if not update or update:
            validLines = [int(line.split()[0]) for line in allLines] #lineId is the first row of the file
        else:
            validLines = [i for i in range(len(allLines))]
    nxtTarget = 0
    print(len(validLines))
    with open(csvFile) as f:
        allLines = list(map(str.strip, f.readlines()))
        print(len(allLines))
        #Conversion x,y,z,qx,qy,qz,qw <--> 4x4 SE(3) matrix
        for i in range(len(allLines)):
            print("{} {}".format(i, nxtTarget))
            if nxtTarget < len(validLines) and i == validLines[nxtTarget]:
                _, x,y,z,qx,qy,qz,qw = allLines[i].split()
                nxtTarget += 1 
                mat = np.eye(4)
                mat[0:3,3] = np.array([x,y,z]).transpose()
                mat[0:3, 0:3] = R.from_quat([qx,qy,qz,qw]).as_matrix()
                allPoses.append(mat)

    assert len(allPoses) == len(validLines), len(allPoses)
    return allPoses

def updateJSONFile(origJsonFile, allPoses, nJsonFile, imu2camera, update):

    with open(origJsonFile, 'r') as oldF:
        # Reading data from auto generated file
        data = json.load(oldF)
        nViews, nExtr = [], []
        
        initialIdPose = data["views"][0]["value"]["ptr_wrapper"]["id"]
        imu2camera = np.array(imu2camera).reshape(4,4)[:3,:3]
        print(imu2camera)
        for ii, pose in enumerate(allPoses):
            dataCurPose = {}
            dataCurView = data["views"][ii]
            curIdPose = initialIdPose + ii
            curRot =   np.linalg.inv(pose[:3,:3]) # Conversion pose <---> extrinsic ( + frame transform MVG)
            curCenter =  pose[:3, 3]

            rot_ = [[curRot[0,0], curRot[0,1], curRot[0,2]],
                    [curRot[1,0], curRot[1,1], curRot[1,2]],
                    [curRot[2,0], curRot[2,1], curRot[2,2]]]

            center_ = [curCenter[0], curCenter[1], curCenter[2]]

            # Adding odometry priors
            if not update:
                dataCurView["value"]["ptr_wrapper"]["data"]["use_pose_center_prior"] = True
                dataCurView["value"]["ptr_wrapper"]["data"]["center_weight"] = [1,1,1]
                dataCurView["value"]["ptr_wrapper"]["data"]["center"] = center_ 

            else:
                dataCurPose["key"] = ii
                dataCurPose["value"] = {"rotation":rot_, "center":center_}
                nExtr.append(dataCurPose)
            
            nViews.append(dataCurView)
        print(len(nExtr))
        # Writing back data
        updatedDict = {}
        updatedDict["sfm_data_version"] = data["sfm_data_version"]
        updatedDict["root_path"] = data["root_path"]
        # updatedDict["lidar_root_path"] = data["lidar_root_path"]
        updatedDict["structure"] = data["structure"]
        updatedDict["control_points"] = data["control_points"]
        updatedDict["intrinsics"] = data["intrinsics"]

        if not update:
            updatedDict["views"] = nViews
            updatedDict["extrinsics"] = data["extrinsics"]
        else:
            updatedDict["views"] = nViews #data["views"]
            updatedDict["extrinsics"] = nExtr

        with open(nJsonFile, 'w') as nFile:
            nFile.write(json.dumps(updatedDict, indent=4))
    
    os.remove(origJsonFile)

    return nJsonFile

def printOpenMvgCommands(mvg_dir, nFileJson, output_folder, update):
    print(" ####################################################### ")
    print(" # Copy/paste the following commands in your terminal: #")
    print(" ####################################################### ")
    print("cd "+mvg_dir)
    if not update:
        # Features
        print("./openMVG_main_ComputeFeatures -i "+nFileJson+" -o "+output_folder+ " -p HIGH --numThreads 4")
        # Matching
        print("./openMVG_main_ComputeMatches -i "+nFileJson+" -o "+ output_folder + " -r .85 -m 0 -f 1 -g e")
    else:
        print("./openMVG_main_ComputeStructureFromKnownPoses -i "+ nFileJson + " -o "+output_folder + " -m "+ +" --match_file " + " -b")
    # Structure

if __name__ == "__main__":
    # Parser 
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml_file', type=str, help="YAML file containing the parameters")
    parser.add_argument('output_folder', type=str, help="Folder containing the output files")
    parser.add_argument('mvg_bin', type=str, help="Path to openMVG executables")
    parser.add_argument('--update', action = "store_true", help="Path to openMVG executables")

    resParser = parser.parse_args()

    # Processing
    output = os.path.abspath(resParser.output_folder)
    yamlData = parseYAMLFile(resParser.yaml_file)

    update = resParser.update
    if not update:
        listFilenames = associateIMU2Lidar2Images(yamlData['scans_folder'], yamlData['images_folder'], yamlData['imu_file'], output, yamlData['ground_truth_file'])
    else:
        listFilenames = os.path.join(os.getcwd(), os.path.join(output, "filenames.txt"))

    allPoses = loadAllPoses(yamlData['imu_file'], listFilenames, update)
    generateInitialJSON(resParser.mvg_bin, output,yamlData, listFilenames)
    nJsonFile = updateJSONFile(os.path.join(output, "sfm_data.json"), allPoses, os.path.join(output, "sfm_data_priors.json"), yamlData["extrinsics"]["imu2camera"], update)
    printOpenMvgCommands(resParser.mvg_bin, nJsonFile, output, update)
