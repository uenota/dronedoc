**************************************************************************
LiDARとAMCLを用いた自己位置推定
**************************************************************************
このページではLiDARとAdaptive Monte Carlo Localization (AMCL)を用いて自己位置推定を行います。

Worldファイルを作る
==========================================================================
Gazeboを起動してWorldファイルを作ります。
すでにあるWorldファイルを使う場合はこの手順は必要ありません。

今回はGazeboモデルデータベース（http://models.gazebosim.org/）にあるWillow Garageモデルを利用します。
Willow GarageモデルはInsertタブから追加することができます。
モデルはデータベースから自動でダウンロードされますが、ダウンロードされるまでにしばらく時間がかかります。

以下のようなWorldファイルを作りましょう。

.. image:: imgs/willow_garage.png

自己位置推定用の地図を作る
==========================================================================
今回はTurtlebotを使ってマップを作成します。
`SLAM Map Building with TurtleBot <http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM>`_ を参考にして地図の作成を行います。

必要なパッケージをインストールする
--------------------------------------------------------------------------

.. code-block:: bash

  sudo apt install ros-kinetic-turtlebot-teleop \
                   ros-kinetic-turtlebot-gazebo \
                   ros-kinetic-turtlebot-bringup

必要なノードを起動する
--------------------------------------------------------------------------
今回はGazebo内のモデルの地図を作成するので、シミュレータを起動します.

.. code-block:: bash

  roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=~/catkin_ws/src/px4_sim_pkg/worlds/willow_garage.world

Turtlebotを起動します。

.. code-block:: bash

  roslaunch turtlebot_bringup minimal.launch

Teleopノードを起動します。
使用するコントローラに適したLaunchファイルを使ってください。
キーボードを使うときはkeyboard_teleop.launchファイルを使います。

.. code-block:: bash

  roslaunch turtlebot_teleop logitech.launch

地図作成用のノードを起動します。
今回は `gmapping <http://wiki.ros.org/gmapping>`_ を使って地図を作成します。

.. code-block:: bash

  roslaunch turtlebot_navigation gmapping_demo.launch

地図を保存する
--------------------------------------------------------------------------
コントローラを使用する場合は `デッドマンスイッチ <https://ja.wikipedia.org/wiki/%E3%83%87%E3%83%83%E3%83%89%E3%83%9E%E3%83%B3%E8%A3%85%E7%BD%AE>`_ に割り当てられたボタンを押しながらスティックを動かしてTurtlebotを操作します。

デッドマンスイッチは大抵LBボタンに割り当てられていますが、そうでない場合は次の手順で確認します。

.. TODO: デッドマンボタンの確認方法

Rvizで作成される地図を確認しながら作業を行うと良いでしょう。

Turtlebotを走らせて一通り地図ができたら、`map_server <http://wiki.ros.org/map_server>`_ を使って地図を保存します。

.. code-block:: bash

  rosrun map_server map_saver -f /tmp/my_map

AMCLを使って自己位置推定を行う
==========================================================================

.. TODO: http://wiki.ros.org/amcl

参考
==========================================================================
`How to Build a Map Using Logged Data - ROS Wiki <http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData>`_
  gmappingを使って自己位置推定用の地図を作る