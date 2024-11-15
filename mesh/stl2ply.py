#!/usr/bin/env /usr/bin/python3

import open3d as o3d
import numpy as np
import copy
from scipy.spatial.transform import Rotation as rot

def samp(name,res):
  mesh = o3d.io.read_triangle_mesh(name+".stl")
  bbox=mesh.get_oriented_bounding_box()
  sz=np.array(bbox.extent)
  points=2*int(sz[0]/res*sz[1]/res)+2*int(sz[1]/res*sz[2]/res)+2*int(sz[2]/res*sz[0]/res)
  print(name+".stl","samples",points)
  pcd=mesh.sample_points_uniformly(number_of_points=points)
  pcdd=pcd.voxel_down_sample(voxel_size=res)
  print(name+".ply","voxels",len(pcdd.points))
  o3d.io.write_point_cloud(name+".ply",pcdd)

samp('circle',0.5)
