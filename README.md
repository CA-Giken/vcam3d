# Virtual 3D Camera

![Screen Shot](screenshot.png)

## 必要ROSパッケージ
### バーチャル2Dカメラ

~~~
git clone https://github.com/lucasw/rviz_camera_stream.git
~~~

### 照明

~~~
git clone https://github.com/mogumbo/rviz_lighting
~~~

### Smart Robot Integrator

~~~
git clone https://github.com/CA-Giken/smabo
~~~

## Rviz設定  

スクリーンショット参照


## Launch
### シーン生成
~~~
roslaunch vcam3d env.launch
~~~

### Rviz起動
~~~
roslaunch vcam3d viewer.launch
~~~

### ハンド起動
~~~
script/hand.py
~~~

