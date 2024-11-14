#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from scipy.spatial.transform import Rotation as R
from smabo import tflib

Config={
  "base_frame_id":"base",
  "frame_id":"tool0_controller",
  "initial_pose":[0,0,1000,180,0,0]
}

def initPose(ev):
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["base_frame_id"]
  tf.child_frame_id=Config["frame_id"]
  tf.transform.translation.x=Config["initial_pose"][0]
  tf.transform.translation.y=Config["initial_pose"][1]
  tf.transform.translation.z=Config["initial_pose"][2]
  euler=Config["initial_pose"][3:6]
  quat=R.from_euler('XYZ',euler,degrees=True).as_quat()
  tf.transform.rotation.x=quat[0]
  tf.transform.rotation.y=quat[1]
  tf.transform.rotation.z=quat[2]
  tf.transform.rotation.w=quat[3]
  if ev is not None: pub_tf.publish(tf)
  return tf.transform

def processFeedback(feedback):
  p = feedback.pose.position
  o = feedback.pose.orientation
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["base_frame_id"]
  tf.child_frame_id=Config["frame_id"]
  tf.transform.translation=p
  tf.transform.rotation=o
  pub_tf.publish(tf)
  print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

if __name__=="__main__":
  rospy.init_node("hand_marker")
    
  pub_tf=rospy.Publisher('/update/config_tf',TransformStamped,queue_size=1);

  server = InteractiveMarkerServer("hand_marker")
    
  # create an interactive marker for our server
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = Config["base_frame_id"]
  int_marker.name = "hand_marker"
  int_marker.description = "Hand"
  int_marker.scale=300
  print(dir(int_marker))
  tr=initPose(None)
  int_marker.pose.position=tr.translation
  int_marker.pose.orientation=tr.rotation

  # create a grey box marker
#  stl_marker = Marker()
#  stl_marker.type = Marker.MESH_RESOURCE
#  stl_marker.mesh_resource = "package://vcam3d/mesh/"+Config["mesh"]+".stl"
#  stl_marker.scale.x = 1
#  stl_marker.scale.y = 1
#  stl_marker.scale.z = 1
#  stl_marker.color.r = 0.5
#  stl_marker.color.g = 0.5
#  stl_marker.color.b = 0.7
#  stl_marker.color.a = 0.5
  # create a non-interactive control which contains the box
#  stl_control = InteractiveMarkerControl()
#  stl_control.always_visible = True
#  stl_control.markers.append( stl_marker )
  # add the control to the interactive marker
#  int_marker.controls.append( stl_control )

  transx_control = InteractiveMarkerControl()
  transx_control.always_visible = True
  transx_control.name = "move_x"
  transx_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(transx_control);

  rotz_control = InteractiveMarkerControl()
  rotz_control.always_visible = True
  rotz_control.name = "rot_z"
  rotz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  rotz_control.orientation.w = 1
  rotz_control.orientation.x = 1
  int_marker.controls.append(rotz_control);

  server.insert(int_marker, processFeedback)
  server.applyChanges()

  rospy.Timer(rospy.Duration(3),initPose,oneshot=True)

  rospy.spin()
