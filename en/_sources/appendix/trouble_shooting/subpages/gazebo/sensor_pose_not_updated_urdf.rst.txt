URDF(xacro)を使った時にセンサーの姿勢が反映されない
=================================================================
症状
-----------------------------------------------------------------
- sensorタグ内で指定したposeタグの内容がモデルに反映されない

もしかして？
-----------------------------------------------------------------
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