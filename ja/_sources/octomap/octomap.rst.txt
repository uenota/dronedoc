:tocdepth: 1

Octomapを使って3Dマップを作る
******************************************************

このページでは、RGBDカメラ付きのドローンと `octomap <http://wiki.ros.org/octomap>`_ パッケージを使用して3Dマップを作成します。

モデルを作成する
======================================================
RGBDカメラ付きのIrisのSDFファイルは以下のとおりです。

.. literalinclude:: ../../models/iris_depth_camera/model.sdf
    :linenos:
    :language: xml
    :caption: model.sdf

以下がRGBDカメラのセンサ部分の定義です。
ROSプラグインについては、`Gazebo plugins in ROS <http://gazebosim.org/tutorials?tut=ros_gzplugins#DepthCamera>`_ を、 ``<camera>`` タグ内の記述については、`gazebo_models / kinect / model.sdf <https://bitbucket.org/osrf/gazebo_models/src/9533d55593096e7ebdfb539e99d2bf9cb1bff347/kinect/model.sdf?at=default&fileviewer=file-view-default>`_ を参考にしました。

.. code-block:: xml
    :linenos:

    <sensor name="camera" type="depth">
        <update_rate>20</update_rate>
        <camera>
          <horizontal_fov>1.047198</horizontal_fov>
          <image>
            <width>640</width>
            <height>320</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.4</near>
            <far>3.5</far>
          </clip>
        </camera>
        <plugin name="camera_plugin" filename="libgazebo_ros_openni_kinect.so">
          <baseline>0.2</baseline>
          <alwaysOn>true</alwaysOn>
          <!-- Keep this zero, update_rate in the parent <sensor> tag
            will control the frame rate. -->
          <updateRate>0.0</updateRate>
          <cameraName>camera_ir</cameraName>
          <imageTopicName>/camera/depth/image_raw</imageTopicName>
          <cameraInfoTopicName>/camera/depth/camera_info</cameraInfoTopicName>
          <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
          <depthImageInfoTopicName>/camera/depth/camera_info</depthImageInfoTopicName>
          <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
          <frameName>camera_link</frameName>
          <pointCloudCutoff>0.05</pointCloudCutoff>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </plugin>
      </sensor>

RGBDカメラが正常に動作するか確認するために、以下のような、ロボットのベースフレームからRGBDカメラへのTFをブロードキャストする設定を書いたLaunchファイル作成します。

.. code-block:: xml
    :linenos:
    :caption: mymodel_sitl_depth_camera.launch

    <launch>

        <node pkg="tf" name="base2depth" type="static_transform_publisher" args="0.1 0 0 -1.57 0 -1.57 base_link camera_link 100"/>

        <arg name="vehicle" default="iris_2d_lidar"/>
        <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world" />
        <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_depth_camera/model.sdf" />
        <arg name="verbose" default="false"/>
        <arg name="debug" default="false"/>

        <include file="$(find px4)/launch/mavros_posix_sitl.launch" >
            <arg name="sdf" value="$(arg sdf)" />
            <arg name="vehicle" value="$(arg vehicle)" />
            <arg name="verbose" value="$(arg verbose)" />
            <arg name="debug" value="$(arg debug)" />
            <arg name="world" value="$(arg world)" />
        </include>

        <param name="/mavros/local_position/tf/send" type="bool" value="true" />
        <param name="/mavros/local_position/frame_id" type="str" value="map" />
        <param name="/mavros/local_position/tf/frame_id" type="str" value="map" />
        <param name="/mavros/setpoint_velocity/mav_frame"  type="str" value="BODY_NED" />

    </launch>

以下のような点群が表示されればOKです。

.. image:: imgs/iris_depth.jpg

Octomapの準備
======================================================
Octomapのインストール
------------------------------------------------------

.. code-block:: bash

    sudo apt install ros-kinetic-octomap-mapping

Launchファイル
------------------------------------------------------
Octomapによる3D地図生成を行うノードである、octomap_server_nodeを実行するためのLaunchファイルを作成します。
octomap_serverパッケージにあるoctomap_mapping.launchファイルを元に変更を加えるので、以下のコマンドでファイルをコピーします。

.. code-block:: bash

    roscp octomap_server octomap_mapping.launch ~/catkin_ws/src/px4_sim_pkg/launch/

編集後のLaunchファイルは以下のとおりです。

今回の設定では、グローバルフレームが ``map`` になっており、点群は ``/camera/depth/points`` にパブリッシュされるので、その部分を変更してあります。

.. literalinclude:: ../../launch/map/octomap_mapping.launch
    :linenos:
    :language: xml
    :caption: octomap_mapping.launch

実行する
------------------------------------------------------
シミュレーション環境を起動します。
今回はwillow_garage.worldをワールドファイルとして使用します。

.. code-block:: bash

    roslaunch px4_sim_pkg mymodel_sitl_depth_cam.launch world:=$HOME/catkin_ws/src/dronedoc/worlds/willow_garage.world

次に、octomap_server_nodeノードを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg octomap_mapping.launch

作成されたマップを見るためにRvizを起動しておきます。

.. code-block:: bash

    rviz

最後に、mavteleopノードを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg mavros_teleop.launch

ゲームパッドから操作できるようにするために、飛行モードをOFFBOARDに変更します。

.. code-block:: bash

    rosrun mavros mavsys mode -c OFFBOARD


ドローンを動かすと、以下のように3Dマップが作成されます。
この動画では ``resolution`` パラメータを変更して地図の解像度を変えてあります。

.. image:: imgs/octomap.gif

参考
======================================================
`OctoMap 3D Models with ROS/Gazebo - PX4 Developer Guide <https://dev.px4.io/en/simulation/gazebo_octomap.html>`_
    PX4 SITL環境でOctomapによるマッピングを行う