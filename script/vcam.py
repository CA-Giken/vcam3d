#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import os
import sys
import subprocess
import copy
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from smabo import open3d_conversions
from smabo import tflib
from smabo.msg import Floats
from rospy.numpy_msg import numpy_msg

Config={
  "scenes":"/config/scene/meshes",
  "frame_id":"camera",
  "width":1000,  #px
  "height":1000,  #px
  "wx":600, #mm
  "wy":600, #mm
  "wd":600, #mm
  "baseline":80, #mm
  "view_r":50000,
}
Param={
  "streaming":True
}


def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def resamp(Plist,Flist):
  trim_far=Config["wd"]
  trim_x=Config["wx"]/2
  trim_y=Config["wy"]/2

  pcds=o3d.geometry.PointCloud()
  for pc,fr in zip(Plist,Flist):
    RT=getRT(Config["frame_id"],fr)
    p=copy.copy(pc)
    p.transform(RT)
    pcds=pcds+p
  scn=np.array(pcds.points)
  zp=np.ravel(scn.T[2])
  scn=scn[zp<trim_far]
  yp=np.ravel(scn.T[1])
  zp=np.ravel(scn.T[2])
  scn=scn[np.abs(yp/zp)<trim_y/trim_far]
  xp=np.ravel(scn.T[0])
  zp=np.ravel(scn.T[2])
  scn=scn[np.abs(xp/zp)<trim_x/trim_far]
  print("vcam trimmed",scn.shape)
  pcds.points=o3d.utility.Vector3dVector(scn)

  if len(pcds.points)>1000:
    _, pm=pcds.hidden_point_removal([0,0,0],Config["view_r"])
    pcds=pcds.select_by_index(pm)
  return pcds

def cb_capture(msg):
  try:
    Param.update(rospy.get_param(""))
  except Exception as e:
    print("get_param exception:",e.args)
  pcd=resamp(Points,Frames)
#
# should publish
#
#
#


def cb_scan(ev):
  try:
    Param.update(rospy.get_param(""))
  except Exception as e:
    print("get_param exception:",e.args)
  if Param["streaming"]:
    pcd=resamp(VoxelPoints,Frames)
    pc2=open3d_conversions.to_msg(pcd,frame_id=Config["frame_id"])
    pub_pc2.publish(pc2)

  dotx=Config["width"]
  doty=Config["height"]
  fx=dotx*Config["wd"]/Config["wx"]
  fy=doty*Config["wd"]/Config["wy"]
  c1x=dotx/2
  c1y=doty/2
  c2x=dotx/2
  c2y=c1y

  cam1=CameraInfo()
  cam1.header.stamp=rospy.Time.now()
  cam1.header.frame_id=Config["frame_id"]
  cam1.width=dotx
  cam1.height=doty
  cam1.K=np.array([fx,0,c1x, 0,fy,c1y, 0,0,1])
  RT1=np.array([1,0,0,0, 0,1,0,0, 0,0,1,0])
  cam1.R=RT1.reshape((3,4))[:3,:3].ravel()
  cam1.P=cam1.K.reshape((3,3)).dot(RT1.reshape((3,4))).ravel()
  pub_info.publish(cam1)

  cam2=CameraInfo()
  cam2.header.stamp=rospy.Time.now()
  cam2.header.frame_id=Config["frame_id"]
  cam2.width=dotx
  cam2.height=doty
  cam2.K=np.array([fx,0,c2x, 0,fy,c2y, 0,0,1])
  RT2=np.array([1,0,0,-Config["baseline"], 0,1,0,0, 0,0,1,0])
  cam2.R=RT2.reshape((3,4))[:3,:3].ravel()
  cam2.P=cam2.K.reshape((3,3)).dot(RT2.reshape((3,4))).ravel()
  pub_info2.publish(cam2)

  rospy.Timer(rospy.Duration(0.5),cb_scan,oneshot=True)

def loadMesh(mesh,frame='world'):
  global Points,Frames,VoxelPoints
  tags=[s for s in mesh.split('/') if len(s)>0]
  tags[-1]=tags[-1].split('.')[0]
  if tags[0].startswith('pack'):
    name=subprocess.getoutput("rospack find "+tags[1])
    for s in tags[2:]: name=name+'/'+s    
  else:
    name=''
    for s in tags: name=name+'/'+s
  name=name+'.ply'
  pcd=o3d.io.read_point_cloud(name)
  print("vcam load ply",name,len(pcd.points))
  voxel=np.sqrt(Config["wx"]*Config["wy"]/50000)
  print("vcam voxel",voxel)
  dwnpcd=pcd.voxel_down_sample(voxel)
  if len(pcd.points)>0:
    VoxelPoints.append(dwnpcd)
    Points.append(pcd)
    Frames.append(frame)

########################################################
rospy.init_node("vcam",anonymous=True)
thispath= subprocess.getoutput("rospack find vcam3d")
###Load params
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print("get_param exception:",e.args)
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)

VoxelPoints=[]
Frames=[]
Points=[]
for m in rospy.get_param(Config["scenes"]):
  loadMesh(m[0],m[1])

###Topics
rospy.Subscriber("~X1",Bool,cb_capture)
pub_pc2=rospy.Publisher("~pc2",PointCloud2,queue_size=1)
pub_floats=rospy.Publisher("~floats",numpy_msg(Floats),queue_size=1)
pub_done=rospy.Publisher("~Y1",Bool,queue_size=1)
pub_info=rospy.Publisher("~info",CameraInfo,queue_size=1)    #the 1st-camera info
pub_info2=rospy.Publisher("~info2",CameraInfo,queue_size=1)  #the 2nd-camera info

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

rospy.Timer(rospy.Duration(3),cb_scan,oneshot=True)

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
