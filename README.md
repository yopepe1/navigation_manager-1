# simple_commander_for_foxy
このリポジトリはNav2のsimple_commanderをfoxyで使えるように移植した</br>
https://github.com/emiraykin/Nav2-Simple-Commander-for-Foxy-and-below</br>
からフォークしたものです。

## ビルド
以下のパッケージをインストールします。
- turtlebot3(https://github.com/ROBOTIS-GIT/turtlebot3)
- turtlebot3_simulation(https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

コードをワークスペースにクローンし、colcon build してください。
```
$ cd your/colcon_workspace/src
$ git clone https://github.com/CuboRex-Development/simple_commander_for_foxy.git
$ cd ../..
$ colcon build --symlink-install
```

## Waypointの読み込み
実行ファイルのpythonファイルと同じディレクトリに'output.csv'があるとそれを読み込みます。
複数のWaypointを登録しておき、連続してNav2に対してWaypointを出力し続けます。
csvファイルの出力形式は以下のとおりです。
https://github.com/CuboRex-Development/simple_commander_for_foxy/blob/a68ba9bb90b2a786f25809ea7344bd2c528d50ac/waypoints/waypoints.csv#L1-L4


waypointはlaunchファイルで指定できます。</br>
場所はsimple_commander_for_foxy/waypoints内に保存して、必ずcolcon buildしてください。ファイルがコピーされて読み込めるようになります。（まだシンボリックリンクではありません）</br>
https://github.com/CuboRex-Development/simple_commander_for_foxy/blob/a68ba9bb90b2a786f25809ea7344bd2c528d50ac/launch/simple_commander_launch.py#L6-L8
'waypoint_flle'の名前を変更してcolcon buildをしてからlaunch.pyを実行してください。

## 実行
シミュレーション環境とNavigation2を起動します。実環境のロボットに使用する場合は不要です。

### 端末1
```
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 端末2
```
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### 端末3
#### 推奨
```
$ ros2 launch simple_commander_for_foxy simple_commander_launch.py
```

#### デフォルト設定での起動
```
$ ros2 run simple_commander_for_foxy simple_commander
```
シミュレーション環境内で巡回を開始します。
![Screenshot from 2023-10-04 10-06-28](https://github.com/CuboRex-Development/simple_commander_for_foxy/assets/22425319/ff932289-ac8d-45b8-9e4c-cbb7d6f64cfe)
