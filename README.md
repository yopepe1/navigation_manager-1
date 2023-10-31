# navigation_manager
navigation2に対して、waypointを投げたり、特定の位置で停止したり、速度を指示したりします。

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
waypointsディレクトリに'waypoint.csv'があるとcolcon build時に実行ディレクトリcolcon_ws/install/pkgs/shareにコピーされ読み込むことができます。
複数のWaypointを登録しておき、連続してNav2に対してWaypointを出力し続けます。
csvファイルの出力形式は以下のとおりです。
[https://github.com/CuboRex-Development/simple_commander_for_foxy/blob/a68ba9bb90b2a786f25809ea7344bd2c528d50ac/waypoints/waypoints.csv#L1-L4](https://github.com/CuboRex-Development/navigation_manager/blob/main/waypoints/waypoints.csv)


waypointはlaunchファイルで指定できます。</br>
場所はsimple_commander_for_foxy/waypoints内に保存して、必ずcolcon buildしてください。ファイルがコピーされて読み込めるようになります。（まだシンボリックリンクではありません）</br>
[https://github.com/CuboRex-Development/simple_commander_for_foxy/blob/a68ba9bb90b2a786f25809ea7344bd2c528d50ac/launch/simple_commander_launch.py#L6-L8](https://github.com/CuboRex-Development/navigation_manager/blob/e776a2e8cdd69fe6bb6a203676425d441efbc70c/launch/navigation_manager_launch.py#L8)
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
$ ros2 launch navigation_manager navigation_manager_launch.py
```

#### デフォルト設定での起動
```
$ ros2 run navigation_manager navigation_manager
```
シミュレーション環境内で巡回を開始します。
![Screenshot from 2023-10-04 10-06-28](https://github.com/CuboRex-Development/simple_commander_for_foxy/assets/22425319/ff932289-ac8d-45b8-9e4c-cbb7d6f64cfe)

## 設定できるフラグ
### 停止フラグ
csvの10列目のstop_flagを1に設定すると、そのwaypointに到着したあと、nキーが押されるまで次のwaypointを送信しません。人の指示があるまでその場で待ち続けます。
写真

### スキップフラグ
csvの11列目のskip_flagを0に設定すると、そのwaypointに到着せずAbortした場合でも、その次のwaypointを目指さず、同じwaypointを目指します。
写真

