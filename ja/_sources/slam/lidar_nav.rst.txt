:tocdepth: 1

LiDARを用いた自律移動
*********************************************************************************

この章では、move_baseとhector_slamを用いて2D LiDARを搭載したドローンを自律移動させます。

move_baseはドローンの制御と経路計画、障害物回避に用い、hector_slamはドローンの自己位置推定に使われます。
hector_slamを使用しているので、同時に地図を作成することもできます。

PX4 SITLシミュレーションを起動する
=================================================================================
:doc:`localization_and_mapping` で使用したmymodel_sitl_hector.launchと内容はほぼ同じですが、今回はmove_baseを使用するため、 ``map`` から ``odom`` へのTFを定義してあります。
``map`` からドローンのベースフレーム（ ``base_link`` ）へのTFはhector_mappingノードがブロードキャストし、local_costmapが使用する ``odom`` から ``base_link`` へのTFはこのLaunchファイル内のstatic_tf_publisherがブロードキャストします。

.. code-block:: xml
    :linenos:
    :caption: mymodel_sitl_lidar_nav.launch

    <launch>

        <node pkg="tf" name="base2lidar" type="static_transform_publisher" args="0 0 0.1 0 0 0 base_link lidar_link 100"/>
        <node pkg="tf" name="map2odom" type="static_transform_publisher" args="0 0 0 0 0 0 map odom 100"/>

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


実行する
=========================================================================

.. code-block:: bash

    roslaunch px4_sim_pkg mymodel_sitl_hector.launch

はじめにシミュレーション環境を起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg mapping_default.launch

``map`` から ``base_link`` へのTFを提供するために自己位置推定用のhector_mappingノードを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg navigation.launch

move_baseノードを起動します。

.. code-block:: bash

    rviz

rvizを使ってロボットに目標姿勢を指示します。

.. code-block:: bash

    rosrun mavros mavsys mode -c OFFBOARD

目標姿勢を指示したら、飛行モードをOFFBOARDに変更して、move_baseのパブリッシュする速度目標値をもとに移動できるようにします。

実際に実行すると、以下のように2D LiDARを用いて推定した自己位置を元にして自律移動させることができます。

.. image:: imgs/lidar_nav.gif