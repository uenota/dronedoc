==================================================================================
トラブルシューティング実例集
==================================================================================
このページではトラブルと考えられる原因をまとめています。

ROS関連
==================================================================================

Gazeboシミュレーションで加速度の値がおかしい
----------------------------------------------------------------------------------
症状
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- 動いていないのに加速度の値が激しく変化する
- 動いていないのにz方向の加速度が重力加速度の値から著しく外れている

もしかして？
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
collision要素にメッシュを指定していませんか？

複雑なcollision要素の指定はこうした問題を引き起こすことがあります。

特に必要でなければ直方体や円柱などシンプルな形状を指定しましょう。

URDF(xacro)を使った時にセンサーの姿勢が反映されない
----------------------------------------------------------------------------------
症状
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- sensorタグ内で指定したposeタグの内容がモデルに反映されない

もしかして？
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
センサーが取り付けられているlinkのjointがfixedになっていませんか？

fixedであるjointは、GazeboがURDFをロードするときに自動で親linkと一体化されてしまうので、センサーが参照しているlinkがなくなります。

センサーの付いているlinkはrevoluteにすることで自動的に一体化されるのを防げます。

例:

.. code-block:: xml

  <joint name="lidar_joint" type="revolute">
    <origin xyz="0 0.01 0.026" rpy="0 0 3.14" />
    <child link="lidar_link"/>
    <parent link="base_link" />
    <limit upper="0" lower="0" effort="0" velocity="0" />
  </joint>
  <!-- Sensor -->
  <gazebo reference="lidar_link">
    <material>Gazebo/DarkGrey</material>
    <sensor name="laser" type="ray">
      <pose>0 -0.03 0.055 0 0 -1.57</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-2.0944</min_angle>
            <max_angle>2.0944</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="LaserPlugin" filename="libgazebo_ros_laser.so">
        <topicName>/laser/scan</topicName>
        <frameName>/lidar_link</frameName>
      </plugin>
      <always_on>1</always_on>
      <update_rate>40</update_rate>
      <visualize>true</visualize>
    </sensor>
  </gazebo>


CMake関連
==================================================================================

Python関連
==================================================================================

import: not authorized `rospy' @ error/constitute.c/WriteImage/1028.
----------------------------------------------------------------------------------
症状
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Pythonで書かれたROSノードを実行すると以下のようなエラーが出る。

.. code-block:: none

  import: not authorized `rospy' @ error/constitute.c/WriteImage/1028.
  from: can't read /var/mail/nav_msgs.msg
  from: can't read /var/mail/geometry_msgs.msg
  import: not authorized `tf' @ error/constitute.c/WriteImage/1028.
  /home/uenot/catkin_ws/src/dronedoc/script/odom_publisher.py: line 8: syntax error near unexpected token `('
  /home/uenot/catkin_ws/src/dronedoc/script/odom_publisher.py: line 8: `local_pos = PoseStamped()'

もしかして？
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Pythonプログラムの先頭に ``#!/usr/bin/env python`` と書くのを忘れていませんか？

`書いてあるのにエラーが出る場合 <https://answers.ros.org/question/306065/error-on-import-ros-packages/>`_

C++関連
==================================================================================

その他
==================================================================================