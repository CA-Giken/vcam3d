#!/usr/bin/env python

import math
from typing import List
import PySimpleGUI as sg
import rospy
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform
import uuid
import os
import yaml
import tf2_ros
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('vcam3d')

topic_publish = "/update/config_tf"
############################
### ユーティリティ start ###
############################

# 条件に合致する最初の要素のインデックスを見つける
def find_first_index(lst, key, value):
    for index, item in enumerate(lst):
        if item[key] == value:
            return index
    return None 

def dict2tf(d):
  tf=Transform()
  tf.translation.x=d['translation']['x']
  tf.translation.y=d['translation']['y']
  tf.translation.z=d['translation']['z']
  tf.rotation.x=d['rotation']['x']
  tf.rotation.y=d['rotation']['y']
  tf.rotation.z=d['rotation']['z']
  tf.rotation.w=d['rotation']['w']
  return tf

def tf2dict(tf):
  d={'translation':{'x':0,'y':0,'z':0},'rotation':{'x':0,'y':0,'z':0,'w':0}}
  d['translation']['x']=float(tf.translation.x)
  d['translation']['y']=float(tf.translation.y)
  d['translation']['z']=float(tf.translation.z)
  d['rotation']['x']=float(tf.rotation.x)
  d['rotation']['y']=float(tf.rotation.y)
  d['rotation']['z']=float(tf.rotation.z)
  d['rotation']['w']=float(tf.rotation.w)
  return d

################################
### PySimpleGUI UI定義 start ###
################################

def build():
    contents = [
        [
            sg.Tree(key="-tree-", data=sg.TreeData(), headings=buildTreeHeadings(), enable_events=True, show_expanded=True, col0_width=24, auto_size_columns=False, col_widths=[6, 6, 6])
        ], 
        [
            sg.Button(button_text="Add", key="-add-"), sg.Button(button_text="Edit", key="-edit-"), sg.Button(button_text="Remove", key="-remove-")
        ]
    ]
    
    return contents

def posTable(readonly: bool):
    table = [
        sg.Column([[sg.Text()], [sg.Text(text="Transform", size=(8, 1))], [sg.Text(text="Rotation", size=(8, 1))]]),
        sg.Column([[sg.Text(text="X")], [sg.InputText(key="x", size=(6, 1), readonly=readonly)], [sg.InputText(key="rx", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y", size=(6, 1), readonly=readonly)], [sg.InputText(key="ry", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z", size=(6, 1), readonly=readonly)], [sg.InputText(key="rz", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="W")], [sg.InputText(key="", size=(6, 1), readonly=readonly)], [sg.InputText(key="rw", size=(6, 1), readonly=readonly)]])
    ], 
    return table

def buildTreeHeadings():
    return ["X", "Y", "Z"]

def buildTree(records: "list[Record]"):
    treeData = sg.TreeData()
    for record in records:
        treeData.insert(record.parentKey, record.key, record.label, [str(record.tfs.transform.translation.x), str(record.tfs.transform.translation.y), str(record.tfs.transform.translation.z)])
    return treeData

def buildAdd():
    contents = [
        posTable(readonly=False),
        [
            sg.Text(text="ラベル"), sg.InputText(key="_label_", size=(15, 1))  
        ],
        [
            sg.Button(button_text="Confirm", key="-add-confirm-"), sg.Button(button_text="Cancel", key="-add-cancel-")
        ]
    ]
    return contents

def buildEdit():
    contents = [
        posTable(readonly=False),
        [
            sg.Text(text="ラベル"), sg.InputText(key="_label_")  
        ],
        [
            sg.Button(button_text="Confirm", key="-edit-confirm-"), sg.Button(button_text="Cancel", key="-edit-cancel-")
        ]
    ]
    return contents

def buildRemove():
    contents = [
        [
            sg.Text(text="この視点を削除します。")
        ],
        [
            sg.Button(button_text="Confirm", key="-remove-confirm-"), sg.Button(button_text="Cancel", key="-remove-cancel-")
        ]
    ]
    return contents

def validateForm(*args):
    # 今のところはパラメータをFloat型に変換できるかのバリデーションのみ
    for arg in args:
        try:
            float(arg)
            continue
        except:
            return False
    return True

######################
### Rviz連携 start ###
######################

class Record(dict):
    def __init__(self, parentKey: str, key: str, label: str, tfs: TransformStamped, isLabel = False):
        super().__init__()
        self.__dict__ = self
        self.parentKey = parentKey
        self.key = key
        self.label = label
        self.isLabel = isLabel
        
        self.tfs: TransformStamped = tfs
        self.tfs.header.frame_id = "base"
        self.tfs.child_frame_id = "tool0_controller"

class TransformManager:
    def __init__(self):
        # 状態管理
        self.records: List[Record] = list()
        self.tempTfs = TransformStamped()
                
        # ROS管理
        rospy.init_node("tf_manager", anonymous = True)
        self.pub = rospy.Publisher(topic_publish, TransformStamped, queue_size = 1)
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # 設定管理
        self.isAutosave = True
        
    def getRecord(self, key: str):
        return next(filter(lambda record: record.key == key, self.records), None)

    def addRecord(self, record: Record):
        self.records.append(record)
        
        self.autosave()
        return record
    
    def updateRecord(self, record: Record):        
        index = find_first_index(self.records, "key", record.key)
        self.records[index] = record
        
        self.autosave()
        return self.records[index]
    
    def removeRecord(self, key: str):
        index = find_first_index(self.records, "key", key)
        self.records.pop(index)
        
        self.autosave()
        return
    
    def selectRecord(self, key):
        record = next(filter(lambda record: record.key == key, self.records))
        if record is None:
            print("Missing record index.")
            return
        if record.isLabel:
            print("It is a label.")
            return 
        record.tfs.header.stamp = rospy.Time.now()
        self.pub.publish(record.tfs)
        print("Published at: ", topic_publish)
        print("Published: ", record.tfs)
        return record
        
    def loadRecords(self, records: List[Record]):
        self.records = records
        return self.records
    
    def loadRecordsFromRosparam(self):
        try:
            list_dict = rospy.get_param("/tf_list")
            records = []
            for dict in list_dict:
                tfs = TransformStamped()
                tfs.transform = dict2tf(dict["tf"])
                record = Record(
                    dict["parentKey"],
                    dict["key"],
                    dict["label"],
                    tfs,
                    dict["isLabel"]
                )
                records.append(record)
            self.records = records
            return self.records
        except Exception as e:
            print(e)
    
    def autosave(self):
        if self.isAutosave:
            self.SaveRecords()
    
    def SaveRecords(self):
        
        records = [
            { 
                "parentKey" : record.parentKey,
                "key": record.key,
                "label": record.label,
                "isLabel": record.isLabel,
                "tf": tf2dict(record.tfs.transform)
            }
            for record in self.records
        ]
        # ROSPARAMにセット
        rospy.set_param("/tf_list", records)
        print("Saved to ROSPARAM /tf_list: ", records)
        # configディレクトリのyamlに書き出し
        filename = os.path.join(package_path, "config", "tfs.yaml")
        yf=open(filename,"w")
        yaml.dump({ "tf_list": records } ,yf,default_flow_style=False)
        print("Saved to", filename)
    
    def updateCurrentCam(self):
        tryCount = 0
        maxTryCount = 10
        while True:
            try:
                tfs = self.tfBuffer.lookup_transform("base", "tool0_controller", rospy.Time())
                self.tempTfs = tfs
                break
            except Exception as e:
                print(e)
                tryCount += 1
                if tryCount > maxTryCount:
                    break
                rospy.sleep(1)
        
    def getLabels(self):
        labelRecords = filter(lambda record: record.isLabel, self.records)
        labelDict = [{ "key": record.key, "label": record.label } for record in labelRecords ]
        return labelDict
    
##########################
### UIとRviz連携 start ###
##########################

if __name__ == "__main__":
    app = TransformManager()
    
    contents = build()
    window = sg.Window("InteractiveMarker Manager", contents, finalize=True)
    
    # # ツリーヘッダー消去
    # window['-tree-'].Widget['show'] = 'tree'
    
    app.loadRecordsFromRosparam()
    treeData = buildTree(app.records)
    window["-tree-"].update(treeData)

    window["-tree-"].bind('<Double-1>', "double-click-")
    
    popup = None
    keyForPopup = ""
    
    while True:
        event, values = window.read(timeout=100, timeout_key='-timeout-')    
        popupEvent, popupValues = popup.read(timeout=100, timeout_key='-timeout-popup-') if not popup is None or popup else (None, None)

        if event == sg.WIN_CLOSED:
            print('exit')
            break
        
        # ツリー操作
        ## 選択中のCamのキーを格納
        if len(values["-tree-"]) == 0: 
            keyForPopup = ""
        else: 
            keyForPopup = values["-tree-"][0]
        
        # ダブルクリックでカメラプリセットへカメラ移動
        if event == '-tree-double-click-':
            # ツリー内のアイテム未選択時は反応しない
            if len(values["-tree-"]) == 0: continue 
            
            key = values["-tree-"][0]
            app.selectRecord(key)
            continue
        
        ##
        ## 新規登録ポップアップ
        ##
        if event == '-add-':
            # popup.close()
            app.updateCurrentCam()

            popup = sg.Window("視点登録", buildAdd(), finalize=True)
            popup["x"].update(str(app.tempTfs.transform.translation.x))
            popup["y"].update(str(app.tempTfs.transform.translation.y))
            popup["z"].update(str(app.tempTfs.transform.translation.z))
            popup["rx"].update(str(app.tempTfs.transform.rotation.x))
            popup["ry"].update(str(app.tempTfs.transform.rotation.y))
            popup["rz"].update(str(app.tempTfs.transform.rotation.z))
            popup["rw"].update(str(app.tempTfs.transform.rotation.w))
            continue
        
        if popupEvent == '-add-confirm-':
            if not validateForm(popupValues["x"], popupValues["y"], popupValues["z"], popupValues["rx"], popupValues["ry"], popupValues["rz"], popupValues["rw"]):
                sg.popup_error('無効な値が入力されています。')
                continue
            newTfs = TransformStamped()
            newTfs.transform.translation = Vector3(float(popupValues["x"]), float(popupValues["y"]), float(popupValues["z"]))
            newTfs.transform.rotation = Quaternion(float(popupValues["rx"]), float(popupValues["ry"]), float(popupValues["rz"]), float(popupValues["rw"]))
            created = Record(
                "", # parentKeyは今後使うかも
                uuid.uuid4().hex,
                popupValues["_label_"],
                newTfs
            )
            app.addRecord(created)
            popup.close()
            popup=None
            treeData = buildTree(app.records)
            window["-tree-"].update(treeData)
            continue
        if popupEvent == '-add-cancel-':
            popup.close()
            popup=None
            continue
        
        ##
        ## 編集ポップアップ
        ##            
        if event == '-edit-':
            # ツリー内のアイテム未選択時は反応しない
            if len(values["-tree-"]) == 0: continue 
            
            record = app.getRecord(keyForPopup)
            # popup.close()
            popup = sg.Window("視点編集", buildEdit(), finalize=True)
            popup["_label_"].update(record.label)
            popup["x"].update(str(record.tfs.transform.translation.x))
            popup["y"].update(str(record.tfs.transform.translation.y))
            popup["z"].update(str(record.tfs.transform.translation.z))
            popup["rx"].update(str(record.tfs.transform.rotation.x))
            popup["ry"].update(str(record.tfs.transform.rotation.y))
            popup["rz"].update(str(record.tfs.transform.rotation.z))
            popup["rw"].update(str(record.tfs.transform.rotation.w))
            continue
        if popupEvent == '-edit-confirm-':
            if not validateForm(popupValues["x"], popupValues["y"], popupValues["z"], popupValues["rx"], popupValues["ry"], popupValues["rz"], popupValues["rw"]):
                sg.popup_error('無効な値が入力されています。')
                continue
            
            newTfs = TransformStamped()
            newTfs.transform.translation = Vector3(float(popupValues["x"]), float(popupValues["y"]), float(popupValues["z"]))
            newTfs.transform.rotation = Quaternion(float(popupValues["rx"]), float(popupValues["ry"]), float(popupValues["rz"]), float(popupValues["rw"]))
                    
            updated = Record(
                "", # parentKeyは今後使うかも 
                keyForPopup,
                popupValues["_label_"],
                newTfs
            )
            app.updateRecord(updated)
            popup.close()
            popup=None
            treeData = buildTree(app.records)
            window["-tree-"].update(treeData)
            continue
        if popupEvent == '-edit-cancel-':
            popup.close()
            popup=None
            continue
        
        ##
        ## 削除ポップアップ
        ##
        if event == '-remove-':
            # ツリー内のアイテム未選択時は反応しない
            if len(values["-tree-"]) == 0: continue 

            # popup.close()
            popup = sg.Window("確認", buildRemove(), finalize=True)
            continue
        if popupEvent == '-remove-confirm-':
            app.removeRecord(keyForPopup)
            popup.close()
            popup=None
            treeData = buildTree(app.records)
            window["-tree-"].update(treeData)
            continue
        if popupEvent == '-remove-cancel-':
            popup.close()
            popup=None
            continue
        
    window.close()