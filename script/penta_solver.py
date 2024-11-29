#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

ObjectPoints=np.array([
  [0,0,0],
  [150,0,0],
  [150*np.cos(np.pi*2/5),150*np.sin(np.pi*2/5),0],
  [150*np.cos(np.pi*4/5),150*np.sin(np.pi*4/5),0],
  [150*np.cos(np.pi*6/5),150*np.sin(np.pi*6/5),0],
  [150*np.cos(np.pi*8/5),150*np.sin(np.pi*8/5),0]])

Kmat=np.array([
  [1080,0,540],
  [0,1080,540],
  [0,0,1]])

Dmat=np.zeros(5)

def solve(img):
  imgry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  ret, imbin = cv2.threshold(imgry, 150, 255, cv2.THRESH_BINARY)
  contours, hierarchy = cv2.findContours(imbin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  if(len(contours)!=7):
    print("Recognised something wrong")
    return np.zeros((4,4)),-1,img
  centers=[]
  xn=-1
  for c,h in zip(contours,hierarchy[0]):
    if h[3]==-1:  #the 1st hierarchy
      e=cv2.fitEllipse(c)
      centers.append(e[0])
    if h[2]>=0:  #has child
      xn=len(centers)-1
#  print(xn,hierarchy[0][xn])
  centers=np.array(centers)
  cog=np.mean(centers,axis=0)
  cox=centers[xn]
  centers=np.delete(centers,xn,0)
  errs=np.linalg.norm(centers-cog,axis=1)
  cn=np.argmin(errs)
  coc=centers[cn]
  centers=np.delete(centers,cn,0)
  axis0=cox-coc
  axis0=axis0/np.linalg.norm(axis0)
#  print(axis0)
  vecs=centers-coc
#  print(vecs)
  nrms=np.linalg.norm(vecs,axis=1)
  vecs=(vecs.T/nrms).T
  inrs=np.inner(axis0,vecs)
  crss=np.cross(axis0,vecs)
  args=np.arccos(inrs)
  args=np.array([a if s>0 else 2*np.pi-a for a,s in zip(args,crss)])
#  print(args)
  centers=centers[args.argsort()]
  centers=np.insert(centers,0,cox,axis=0)
  centers=np.insert(centers,0,coc,axis=0) 
#  print(centers)

  cv2.drawContours(img, contours, -1, (0,0,255), 3)
  for i,cod in enumerate(centers):
    cv2.putText(img,"#"+str(i),(int(cod[0]),int(cod[1])), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0,0,255))

  sol,rvec,tvec=cv2.solvePnP(ObjectPoints,centers,Kmat,Dmat)
  if not sol: return np.zeros((4,4)),-1,img

  cTt=np.eye(4)
  cTt[:3,:3]=Rot.from_rotvec(rvec.ravel()).as_matrix()
  cTt[:3,3]=tvec.ravel()

  Pmat=Kmat.dot(cTt[:3])
  pvec=Pmat.dot(np.insert(ObjectPoints.T,3,1,axis=0))
  uvs=(pvec[:2]/pvec[2]).T
  err=np.linalg.norm(uvs-centers,axis=1)
#  print(err)
  return cTt,np.mean(err),img


test_datas=[
  ['../img/penta000.jpg',[0,0,500,0,0,0,1]],
  ['../img/penta001.jpg',[10.5952,0,500,-0.026529,0.069603,-0.35517,0.93183]]]

if __name__=="__main__":

  for data in test_datas:  
    img = cv2.imread(data[0],cv2.IMREAD_COLOR)
    cTt,err,img=solve(img)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    quat=Rot.from_matrix(cTt[:3,:3]).as_quat()
    print("Truth",data[0],data[1])
    print("Estimated",cTt[:3,3],quat,err)
    terr=np.linalg.norm(cTt[:3,3]-np.array(data[1][:3]))
    rerr=np.linalg.norm(Rot.from_quat(quat).as_rotvec()-Rot.from_quat(data[1][3:]).as_rotvec())
    print("Estimation error",terr,np.rad2deg(rerr))
  cv2.destroyAllWindows()

