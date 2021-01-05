:tocdepth: 1

Gmappingを使って地図を生成する
=================================================================
このページではTurtlebotとgmappingを用いてGazeboシミュレーション内の地図を生成する方法について扱います。

Worldファイルを作る
-----------------------------------------------------------------
Gazeboを起動してWorldファイルを作ります。
すでにあるWorldファイルを使う場合はこの手順は必要ありません。

今回はGazeboモデルデータベース（http://models.gazebosim.org/）にあるWillow Garageモデルを利用します。
Willow GarageモデルはInsertタブから追加することができます。
モデルはデータベースから自動でダウンロードされますが、ダウンロードされるまでにしばらく時間がかかります。

以下のようなワールドを作り、Worldファイルとして保存します。

.. image:: imgs/willow_garage.png

地図を作る
-----------------------------------------------------------------
`SLAM Map Building with TurtleBot <http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM>`_ を参考にして地図の作成を行います。

必要なパッケージをインストールする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash

  sudo apt install ros-kinetic-turtlebot-teleop \
                   ros-kinetic-turtlebot-gazebo \
                   ros-kinetic-turtlebot-bringup

必要なノードを起動する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
今回はGazebo内のモデルの地図を作成するので、シミュレータを起動します.

.. code-block:: bash

  roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/px4_sim_pkg/worlds/willow_garage.world

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Rvizで作成される地図を確認しながら作業を行うと良いでしょう。

.. code-block:: bash

  roslaunch turtlebot_rviz_launchers view_navigation.launch

.. image:: imgs/mapping.gif

Turtlebotを走らせて一通り地図ができたら、`map_server <http://wiki.ros.org/map_server>`_ を使って地図を保存します。
以下のコマンドを実行するとホームディレクトリ以下に次の２つのファイルが生成されます。

my_map.pgm
  グリッドの占有状態を示す画像ファイル
my_map.yaml
  マップのメタデータが記述されているファイル

.. code-block:: bash

  rosrun map_server map_saver -f ~/my_map

マップファイルのフォーマットについては `Map Format <http://wiki.ros.org/map_server>`_ を参照してください。


参考
-----------------------------------------------------------------
`How to Build a Map Using Logged Data - ROS Wiki <http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData>`_
  gmappingを使って自己位置推定用の地図を作る