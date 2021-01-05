:tocdepth: 1

Turtlebotを使ってマップを作る
*********************************************************************************
このページではTurtlebotと `costmap_2d <http://wiki.ros.org/costmap_2d>`_ を用いてGazeboシミュレーション内の地図を生成する方法について扱います。
ドローンを使った場合には、姿勢の傾きによって正確な地図が作成できない恐れがあるので、今回はTurtlebotを使用します。

costmap_2dパッケージは、ロボットのナビゲーションに用いられる占有格子地図を生成するノードを提供するパッケージです。
占有格子地図はセンサ情報をもとに計算された障害物の存在確率を示す地図で、さらにロボットの形状や障害物の情報を考慮することでコストマップが生成されます。

costmap_2dパッケージについてはROS Wikiページに詳細な解説があるので参考にしてください。

Worldファイルを作る
==========================================================================
Gazeboを起動してWorldファイルを作ります。
すでにあるWorldファイルを使う場合はこの手順は必要ありません。

今回はGazeboモデルデータベース（http://models.gazebosim.org/）にあるWillow Garageモデルを利用します。
Willow GarageモデルはInsertタブから追加することができます。
モデルはデータベースから自動でダウンロードされますが、ダウンロードされるまでにしばらく時間がかかります。

以下のようなワールドを作り、Worldファイルとして保存します。

.. image:: imgs/willow_garage.png

地図作成用のノードを作成する
--------------------------------------------------------------------------
地図を保存するmap_saverノードは、 ``/map`` トピックのメッセージをサブスクライブするので、costmap_2dが地図をパブリッシュする ``costmap/costmap`` からリマップする必要があります。

また、コストマップを作成するためにcostmap_2d パッケージのcostmap_2d_nodeを使用します。
パラメータは前回と同様、設定ファイルに記述したものを ``rosparam`` タグ内でロードしています。

.. code-block:: xml
    :linenos:
    :caption: map_builder.launch

    <launch>
        <remap from="/costmap_node/costmap/costmap" to="map" />
        <node pkg="costmap_2d" type="costmap_2d_node" name="costmap_node" respawn="false" output="screen">
        <rosparam command="load" ns="costmap" file="$(find px4_sim_pkg)/config/costmap_params.yaml"/>
        </node>
    </launch>

costmap_2d_nodeのパラメータの設定ファイルは以下のようになっています。

.. literalinclude:: ../../config/costmap_params.yaml
    :linenos:
    :language: yaml
    :caption: costmap_params.yaml

大部分は前章で解説しているので説明は省きますが、以下の点に注意してください。

``inflation_raduis``
  障害物から、この値の距離だけ離れた場所までコストを付加します。これは、ロボットの形状をコストマップに反映するためのものですが、今回は地図を生成したいので、0に設定します。
``publish_frequency``
  地図の表示用に情報をパブリッシュする周波数を設定しています。デフォルトでは0になっており、表示されないので注意してください。
``global_frame``
  今回はTurtlebotのホイールオドメトリを自己位置として用いるので、 ``odom`` に設定しています。
``laser_scan_sensor``
  今回使用するTurtlebotのノードは、RGBDカメラのDepth画像を擬似的なレーザースキャンとしてパブリッシュします。そのフレームが ``camera_depth_frame`` 、トピックが ``/scan`` なので、それぞれ変更します。
``obstacle_range``
  この値の距離以下にある障害物のみが地図に反映されます。今回は地図作成の速度を向上させるために5 mにしてあります。
``rolling_window``
  生成される地図の領域を固定するために ``false`` に設定しています。 ``true`` にした場合にはロボットが移動すると地図の領域も一緒に移動します。また、willow_garage.world内の建物が地図の領域内に収まるように ``origin_x`` と ``origin_y`` も変更しています。

地図の大きさが足りない場合は ``width`` 及び ``height`` パラメータを変更してください。

必要なノードを起動する
--------------------------------------------------------------------------
今回はGazebo内のモデルの地図を作成するので、シミュレータを起動します.

.. code-block:: bash

  roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/px4_sim_pkg/worlds/willow_garage.world

Teleopノードを起動します。
使用するコントローラに適したLaunchファイルを使ってください。
キーボードを使うときはkeyboard_teleop.launchファイルを使います。

.. code-block:: bash

  roslaunch turtlebot_teleop logitech.launch

地図作成用のノードを起動します。

.. code-block:: bash

  roslaunch px4_sim_pkg map_builder.launch


地図を保存する
--------------------------------------------------------------------------
コントローラを使用する場合は `デッドマンスイッチ <https://ja.wikipedia.org/wiki/%E3%83%87%E3%83%83%E3%83%89%E3%83%9E%E3%83%B3%E8%A3%85%E7%BD%AE>`_ に割り当てられたボタンを押しながらスティックを動かしてTurtlebotを操作します。

デッドマンスイッチは大抵LBボタンに割り当てられていますが、そうでない場合は :ref:`deadman` の手順で確認します。

Rvizで作成される地図を確認しながら作業を行うと良いでしょう。
今回は ``odom`` フレームが地図のベースフレームなので、Fixed Frameを ``odom`` に設定して地図を表示させます。

.. code-block:: bash

  rviz

.. image:: imgs/costmap.gif

Turtlebotを走らせて一通り地図ができたら、`map_server <http://wiki.ros.org/map_server>`_ を使って地図を保存します。
以下のコマンドを実行するとホームディレクトリ以下に次の２つのファイルが生成されます。

my_map.pgm
  グリッドの占有状態を示す画像ファイル
my_map.yaml
  マップのメタデータが記述されているファイル

.. code-block:: bash

  rosrun map_server map_saver -f ~/my_map

マップファイルのフォーマットについては `Map Format <http://wiki.ros.org/map_server>`_ を参照してください。

さいごに
=================================================================
今回はTurtlebotのホイールオドメトリを用いて地図を作成しました。
本来はスリップや外乱などの影響で自己位置に誤差が生じるのですが、今回のようなシミュレーションの場合は無視できるので、このような方法で地図を作成することができます。
誤差が無視できない場合に、より高精度に自己位置を推定するために、次章で使うAMCLやカルマンフィルタなどの手法を使います。
また、これらの自己位置推定の手法と地図生成を同時に行うことをSimultaneous Localization and Mapping(SLAM) と言います。

.. _deadman:

補足: デッドマンスイッチを確認する
==========================================================================
``/joy`` トピック
--------------------------------------------------------------------------
teleopノードを起動すると、 ``/joy`` トピックにジョイスティックの入力値がパブリッシュされます。

axesがスティックの入力、buttonsがボタンの入力です。
それぞれインデックスは0から始まるので注意してください。

.. code-block:: none

  $ rostopic echo /joy
  header:
    seq: 2
    stamp:
      secs: 1543249966
      nsecs: 586445147
    frame_id: ''
  axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  ---
  header:
    seq: 3
    stamp:
      secs: 1543249966
      nsecs: 734398094
    frame_id: ''
  axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  buttons: [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
  ---

このように、``/joy`` トピックを確認しながらジョイスティックを操作することでどのようにボタンやスティックがマッピングされているかを確認できます。


logitech.launchを使う場合
--------------------------------------------------------------------------
logitech.launchを使う場合には、デッドマンスイッチは4番のIDを持つボタンに固定されています。
上記の手順でボタンのIDを確認して使用しましょう。

ps3_teleop.launchもしくはxbox360_teleop.launch
--------------------------------------------------------------------------
これらのLaunchファイルを使う場合には、 ``axis_deadman`` パラメータにデッドマンスイッチのIDが格納されています。
以下のコマンドを使えば現在の設定値が確認できます。
パラメータの名前は環境によって異なる場合があるので適宜変更してください。

.. code-block:: bash

  rosparam get /turtlebot_teleop_joystick/axis_deadman

このようにして確認したIDがどのボタンにマッピングされているかを上記の手順で確認しても良いですし、以下のコマンドを使って自分の好きなIDに設定することもできます。

.. code-block:: bash

  rosparam set /turtlebot_teleop_joystick/axis_deadman 4

参考
================================================================
`3.2.1. Occupancy Grid Map <https://ja.coursera.org/lecture/robotics-learning/3-2-1-occupancy-grid-map-0QuFW>`_
  CourseraのRoboticsコース
`自律移動ロボットのためのグリッドマップ作成MATLAB, Pythonサンプルプログラム <https://myenigma.hatenablog.com/entry/20140714/1405343128>`_
  グリッドマップのアルゴリズムとプログラム例