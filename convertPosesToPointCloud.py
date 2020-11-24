'''
    Reads a file containing poses and converts them to a point cloud
    Takes a .csv file (TUM format) as input file and outputs a .pcd file
'''
import argparse
import numpy as np
import open3d as o3d 
from scipy.spatial.transform import Rotation as R

def main(filename, output):
    allGT = []

    with open(filename, 'r') as f:
        allLines = list(map(str.strip, f.readlines()))
        origin = np.array([0, 0, 0, 1]).transpose()
        for line in allLines:
            x,y,z,qx,qy,qz,qw = list(map(float, line.split()[1:]))
            tra = np.array([x,y,z]).transpose()
            rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
            T  = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = tra
            allGT.append((T @ origin)[:3])
    
    curP = o3d.geometry.PointCloud()
    curP.points = o3d.utility.Vector3dVector(allGT)
    o3d.io.write_point_cloud(output, curP)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    parser.add_argument("output")
    res = parser.parse_args()
    main(res.filename, res.output)