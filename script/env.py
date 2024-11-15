#!/usr/bin/env python3

# Python includes
import numpy as np
import subprocess
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# ROS includes
import roslib
import rospy
import tf
import tf2_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from tf import transformations
from rviz_tools_py import rviz_tools
from rospy.numpy_msg import numpy_msg
from rovi_utils import tflib

Config={
  "meshes":[]
}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print('frame not found',ref)
    RT=np.eye(4)
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def cleanup_node():
  print("Shutting down node")
  markers.deleteAllMarkers()

def cb_redraw(msg):
  pass
#  f=Floats()
#  f.data=np.ravel(Points)
#  pub_wp.publish(f)

def showModel(name,frame='world',color=(0.8,0.8,0.8),x=0,y=0,z=0,theta=0):
  global Points
  mesh=name
  wTu=getRT('world',frame)
  scale=Vector3(1,1,1)
  uT=np.eye(4)
  uT[0,3]=x
  uT[1,3]=y
  uT[2,3]=z
  uT[:3,:3]=R.from_euler('Z',theta,degrees=True).as_matrix()
  markers.publishMesh(wTu.dot(uT),mesh,color,scale, 1)

# Initialize the ROS Node
rospy.init_node('vscene', anonymous=False, log_level=rospy.INFO, disable_signals=False)
###Load Config
try:
  Config.update(rospy.get_param("/config/scene"))
except Exception as e:
  print("get_param exception:",e.args)

#pub_wp=rospy.Publisher("/vscene/floats",PointCloud2,queue_size=1)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
rospy.on_shutdown(cleanup_node)

markers=rviz_tools.RvizMarkers('world', 'vscene_marker')

###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

rospy.sleep(2)

while not rospy.is_shutdown():
  for m in Config["meshes"]:
    showModel(m[0],frame=m[1],color=m[2])
  rospy.Rate(3).sleep()
