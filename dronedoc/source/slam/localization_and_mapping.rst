:tocdepth: 1

LiDARを用いたSLAM
*********************************************************************************

SLAM
=================================================================================
:doc:`lidar_localization` では、 :doc:`turtle_mapping` 等で事前に作成した地図を利用して、自己位置推定を行いました。

ノードとトピックの概略図は以下のとおりです。
点線内は事前にマップを作る作業におけるノードの例で、この場合はcostmap_2dを用いた例を示しています。
costmap_2dはセンサ情報を元に作成した占有格子地図を ``/map`` トピックにパブリッシュするので、パブリッシュされた地図をmap_saverノードを用いてファイルに保存します。

map_serverノードはこのようにして作成された既存の地図を読み込んで ``/map`` トピックにパブリッシュします。
amclノードがパブリッシュされた地図の情報とセンサー情報をもとに自己位置推定を行い、TFをブロードキャストします。

.. image:: imgs/amcl_diagram.png

点線内の地図作成と、点線の外の自己位置推定は同時に行われないことに注意してください。
これを同時に行うことをSLAM（Simultaneous Localization and Mapping）と言います。

.. image:: imgs/hector_diagram.png

gmappingとHector SLAM
--------------------------------------------------------------------------------
今回はHector SLAMアルゴリズムを用いて地図生成と自己位置推定を行います。
:doc:`../build_map_gmapping/build_map_gmapping` で使用したgmappingもSLAMアルゴリズムの一つです。

gmappingはロボットのオドメトリが必要である一方で、Hector SLAMはオドメトリフリーなSLAM手法です。
ドローンを用いる場合には、オドメトリを得ることが難しいことが多いので、ドローン用のSLAM手法としては、Hector SLAMが適しているといえます。

ROSパッケージとして提供されているLiDARを用いたSLAMアルゴリズムの比較については、 `ROSのLidarSLAMまとめ - Qiita <https://qiita.com/nnn112358/items/814c0fb0d2075eb71da0>`_ が参考になります。

パッケージのインストール
=================================================================================
`hector_slam <http://wiki.ros.org/hector_slam>`_ パッケージを使用するので、以下のコマンドでインストールしましょう。

.. code-block:: bash

  sudo apt install ros-kinetic-hector-slam

Launchファイル
================================================================================
PX4 SITLシミュレータを起動する
-------------------------------------------------------------------------------
SITLシミュレータを起動するLaunchファイルは以下のとおりです。
mymodel_sitl_amcl.launchとほとんど同じですが、 ``odom`` から ``base_link`` へのTFをパブリッシュする必要が無いので、パラメータを設定する部分を消してあります。

.. code-block:: xml
  :linenos:
  :caption: mymodel_sitl_hector.launch

  <launch>

      <node pkg="tf" name="base2lidar" type="static_transform_publisher" args="0 0 0.1 0 0 0 base_link lidar_link 100"/>

      <arg name="vehicle" default="iris_2d_lidar"/>
      <arg name="world" default="$(find px4_sim_pkg)/worlds/willow_garage.world" />
      <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_2d_lidar/model.sdf" />
      <arg name="verbose" default="false"/>
      <arg name="debug" default="false"/>

      <include file="$(find px4)/launch/mavros_posix_sitl.launch" >
          <arg name="sdf" value="$(arg sdf)" />
          <arg name="vehicle" value="$(arg vehicle)" />
          <arg name="verbose" value="$(arg verbose)" />
          <arg name="debug" value="$(arg debug)" />
          <arg name="world" value="$(arg world)" />
      </include>

      <param name="/mavros/setpoint_velocity/mav_frame"  type="str" value="BODY_NED" />
  </launch>

hector_mappingノードを起動する
-------------------------------------------------------------------------------
`hector_mapping <http://wiki.ros.org/hector_mapping>`_ パッケージのmapping_default.launchをベースに変更を行うので、 ``roscp`` コマンドを使ってファイルをコピーします。

.. code-block:: bash

  roscp hector_mapping mapping_default.launch catkin_ws/src/px4_sim_pkg/launch/

変える必要があるのは、以下の3つの値だけです。

それぞれ、以下の通りなので、自分のロボットに適したように設定します。

``base_frame``
  ロボットのベース座標系（フレーム）
``odom_frame``
  ロボットのオドメトリ座標系（フレーム）
``scan_topic``
  レーザースキャンのパブリッシュされるトピック

また、ロボットのオドメトリが得られない場合は、 ``base_frame`` の値と ``odom_frame`` の値を同じにします。

hector_mappingパッケージを使う際のTFの設定については `How to set up hector_slam for your robot <http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot>`_ に情報があるので、参考にしましょう。

.. code-block:: xml

  <arg name="base_frame" default="base_footprint"/>
  <arg name="odom_frame" default="nav"/>
  <arg name="scan_topic" default="scan"/>

今回は以下のとおりにします。

.. code-block:: xml

  <arg name="base_frame" default="base_link"/>
  <arg name="odom_frame" default="base_link"/>
  <arg name="scan_topic" default="/laser/scan"/>

最終的に、Launchファイルは以下のとおりになります。

.. literalinclude:: ../../launch/map/mapping_default.launch
  :linenos:
  :language: xml
  :caption: mapping_default.launch

実行する
=================================================================================
はじめに、PX4 SITLシミュレータを起動します。

.. code-block:: bash

  roslaunch px4_sim_pkg mymodel_sitl_hector.launch

今回はゲームパッドを用いてドローンを操作するので、mavteleopノードを起動します。

.. code-block:: bash

  roslaunch px4_sim_pkg mavros_teleop.launch

ドローンを離陸させたあとに、hector_mappingノードを起動します。

.. code-block:: bash

  roslaunch px4_sim_pkg mapping_default.launch

Rvizを起動して、地図がきちんとできているか確認しましょう。

.. code-block:: bash

  rviz

最後に、OFFBOARDモードに変更して、ドローンを飛行させてみましょう。

.. code-block:: bash

  rosrun mavros mavsys mode -c OFFBOARD

.. image:: imgs/hector.gif

参考
=================================================================================
`ROS and Hector SLAM for Non-GPS Navigation <http://ardupilot.org/dev/docs/ros-slam.html>`_
  Hector SLAMを使った非GPS環境でのナビゲーション
`A Flexible and Scalable SLAM System with Full 3D Motion Estimation <https://doi.org/10.1109/SSRR.2011.6106777>`_
  Hector SLAMアルゴリズムの論文
`rosbash - ROS Wiki <http://wiki.ros.org/rosbash>`_
  ROS向けbashコマンド
