:tocdepth: 1

ドローンに距離センサを搭載する
==========================================================

距離センサをつける
----------------------------------------------------------
はじめに
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
レーザー高度計（`Lidar Lite <https://docs.px4.io/en/sensor/lidar_lite.html>`_ など）とRayセンサ（Lidarなど）を同時に使用するとセグメンテーションフォールトでGazeboが起動できない（ `#9156 <https://github.com/PX4/Firmware/issues/9156>`_）ので今回は距離センサとして超音波センサを利用します。
GpuRayセンサであれば問題なく起動できるらしいので、どうしてもレーザー高度計を使用したい場合にはGpuRayセンサを利用すると良いでしょう。
ただし、GpuRayセンサはGPUを使用しない場合は正常に動作しないようなので、GPUがある場合に限って使用しましょう。

SDFファイル
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
iris_2d_lidarモデルをインクルードして超音波センサをつけても良いのですが、名前空間が煩雑（iris_2d_lidar::iris::base_linkとか）になり、TFの設定などが面倒なので、iris_2d_lidarモデルのSDFファイルをコピーして超音波センサを追加します。

超音波センサを追加したモデルは以下のようになります。

.. literalinclude:: ../../models/iris_sonar/model.sdf
    :language: xml
    :linenos:
    :caption: model.sdf

configファイル
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
以下のようなmodel.configも作成し、modelsディレクトリ以下にiris_sonarディレクトリを作ってその中に保存しましょう。
authorタグの中身は適宜変更してください。

.. literalinclude:: ../../models/iris_sonar/model.config
    :language: xml
    :linenos:
    :caption: model.config

モデルを表示する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
以下のようなLaunchファイルを作って、モデルがきちんと作成されているか確認してみましょう。

.. code-block:: xml
    :caption: mymodel_sonar.launch
    :linenos:

    <launch>

        <node pkg="tf" name="base2sonar" type="static_transform_publisher" args="0 0 0 0 -1.57 0 base_link link 100"/>

        <include file="$(find px4_sim_pkg)/launch/mymodel_sitl_tf.launch" >
            <arg name="vehicle" value="iris_2d_lidar"/>
            <arg name="sdf" value="$(find px4_sim_pkg)/models/iris_sonar/model.sdf" />
        </include>

    </launch>

Launchファイルを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg mymodel_sonar.launch

地上にいる時はわかりませんが、離陸するときちんと超音波センサが取り付けられていることがわかります（青い円錐）。

.. image:: imgs/sonar.png

距離センサの値をパブリッシュする
----------------------------------------------------------------
ドローンに搭載された距離センサの値をパブリッシュするには、 `mavros_extras <http://wiki.ros.org/mavros_extras>`_ パッケージのdistance_sensorプラグインを使用します。

mavros_extrasパッケージには、他にも `px4_flow <https://docs.px4.io/en/sensor/px4flow.html>`_ からのデータをROSトピックにパブリッシュするプラグインなど、追加のセンサ用のプラグインが複数あります。

プラグインをブラックリストから外す
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
デフォルトではdistance_sensorプラグインはブラックリストに入っていて読み込まれないので、ブラックリストから外す必要があります。
プラグインのブラックリストおよびホワイトリストは、px4_pluginlists.yamlファイルに記述されています。
mavrosノードが起動するときに、このファイルの内容をROSパラメータとして読み込むことで、ブラックリストとホワイトリストが反映されます。

mavrosパッケージ以下にあるpx4_pluginlists.yamlを直接編集してもいいのですが、これを変更すると他のLaunchファイルなどに影響を与える可能性があるので、すでにあるファイルをコピーして編集します。
以下のコマンドでmavrosパッケージのディレクトリ内にあるpx4_pluginlists.yamlをコピーします。

.. code-block:: bash

    roscp mavros px4_pluginlists.yaml ~/catkin_ws/src/px4_sim_pkg/config/

以下のようにplugin_blacklist以下のdistance_sensorをコメントアウトします。

.. literalinclude:: ../../config/px4_pluginlists.yaml
    :linenos:
    :language: yaml
    :caption: px4_pluginlists.yaml

パラメータの設定
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
次に、distance_sensor用のROSパラメータを記述するファイルを作成します。

それぞれのパラメータの意味は以下のとおりです。

``sonar_pub``
    センサ名。このセンサからのデータは、``/mavros/distance_sensor/sonar_pub`` にパブリッシュされます。
``id``
    センサのID。QGCのMAVLink InspectorのDISTANCE SENSORの項目から確認できます。
``frame_id``
    センサのベースフレーム名。
``field_of_view``
    センサーの視野（Field of View; FOV）
``send_tf``
    センサのTFを送るかどうか
``sensor_position``
    センサの位置。

.. literalinclude:: ../../config/distance_sensor.yaml
    :linenos:
    :language: yaml
    :caption: distance_sensor.yaml

idは以下の動画のようにして確認できます。

.. image:: imgs/qgc_idcheck.gif

Launchファイルを作る
----------------------------------------------------------------
以下のようなLaunchファイルを作成します。
TFをパブリッシュする設定などは省いているので、move_base等を使用する場合は適宜変更を加えてください。

.. code-block:: xml
    :linenos:
    :caption: mymodel_sonar.launch

    <launch>

        <!-- Static TF Publisher -->
        <node pkg="tf" name="base2sonar" type="static_transform_publisher" args="0 0 -0.04 0 -1.57 0 base_link sonar_model::link 100"/>
        <node pkg="tf" name="base2lidar" type="static_transform_publisher" args="0 0 0.1 0 0 0 base_link lidar_link 100"/>

        <!-- PX4 SITL -->
        <arg name="vehicle" default="iris_2d_lidar"/>
        <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world" />
        <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_sonar/model.sdf" />
        <arg name="verbose" default="false"/>
        <arg name="debug" default="false"/>

        <include file="$(find px4)/launch/posix_sitl.launch" >
            <arg name="sdf" value="$(arg sdf)" />
            <arg name="vehicle" value="$(arg vehicle)" />
            <arg name="verbose" value="$(arg verbose)" />
            <arg name="debug" value="$(arg debug)" />
            <arg name="world" value="$(arg world)" />
        </include>

        <!-- mavros -->
        <arg name="fcu_url" default="udp://:14540@localhost:14557" />
        <arg name="gcs_url" default="" />
        <arg name="tgt_system" default="1" />
        <arg name="tgt_component" default="1" />
        <arg name="log_output" default="screen" />
        <arg name="fcu_protocol" default="v2.0" />
        <arg name="respawn_mavros" default="false" />

        <include file="$(find mavros)/launch/node.launch">
            <arg name="pluginlists_yaml" value="$(find dronedoc)/config/px4_pluginlists.yaml" />
            <arg name="config_yaml" value="$(find dronedoc)/config/distance_sensor.yaml" />

            <arg name="fcu_url" value="$(arg fcu_url)" />
            <arg name="gcs_url" value="$(arg gcs_url)" />
            <arg name="tgt_system" value="$(arg tgt_system)" />
            <arg name="tgt_component" value="$(arg tgt_component)" />
            <arg name="log_output" value="$(arg log_output)" />
            <arg name="fcu_protocol" value="$(arg fcu_protocol)" />
            <arg name="respawn_mavros" default="$(arg respawn_mavros)" />
        </include>

    </launch>

解説
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: xml

    <node pkg="tf" name="base2sonar" type="static_transform_publisher" args="0 0 -0.04 0 -1.57 0 base_link sonar_model::link 100"/>
    <node pkg="tf" name="base2lidar" type="static_transform_publisher" args="0 0 0.1 0 0 0 base_link lidar_link 100"/>

ドローンのベースフレームからLiDARと距離センサへのTFをブロードキャストするためのノードを起動します。

.. code-block:: xml

    <!-- PX4 SITL -->
    <arg name="vehicle" default="iris_2d_lidar"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world" />
    <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_sonar/model.sdf" />
    <arg name="verbose" default="false"/>
    <arg name="debug" default="false"/>

    <include file="$(find px4)/launch/posix_sitl.launch" >
        <arg name="sdf" value="$(arg sdf)" />
        <arg name="vehicle" value="$(arg vehicle)" />
        <arg name="verbose" value="$(arg verbose)" />
        <arg name="debug" value="$(arg debug)" />
        <arg name="world" value="$(arg world)" />
    </include>

PX4 SITLシミュレーションを起動します。
ソナー付きのIrisのモデルを指定しています。

また、今回はmavrosを別で起動するので、mavros_posix_sitl.launchではなく、posix_sitl.launchをインクルードします。

.. code-block:: xml

    <!-- mavros -->
    <arg name="fcu_url" default="udp://:14540@localhost:14557" />
    <arg name="gcs_url" default="" />
    <arg name="tgt_system" default="1" />
    <arg name="tgt_component" default="1" />
    <arg name="log_output" default="screen" />
    <arg name="fcu_protocol" default="v2.0" />
    <arg name="respawn_mavros" default="false" />

    <include file="$(find mavros)/launch/node.launch">
        <arg name="pluginlists_yaml" value="$(find dronedoc)/config/px4_pluginlists.yaml" />
        <arg name="config_yaml" value="$(find dronedoc)/config/distance_sensor.yaml" />

        <arg name="fcu_url" value="$(arg fcu_url)" />
        <arg name="gcs_url" value="$(arg gcs_url)" />
        <arg name="tgt_system" value="$(arg tgt_system)" />
        <arg name="tgt_component" value="$(arg tgt_component)" />
        <arg name="log_output" value="$(arg log_output)" />
        <arg name="fcu_protocol" value="$(arg fcu_protocol)" />
        <arg name="respawn_mavros" default="$(arg respawn_mavros)" />
    </include>

mavrosを起動します。
mavrosパッケージのpx4.launchと内容はほとんど同じですが、``fcu_url`` をシミュレーション用に、``pluginlists_yaml`` と ``config_yaml`` を作成した設定ファイルに変えています。

実行する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
作成したLaunchファイルを実行します。

.. code-block:: bash

    roslaunch px4_sim_pkg mymodel_sonar.launch

距離センサのトピックの値を出力します。

.. code-block:: bash

    rostopic echo /mavros/distance_sensor/sonar_pub

以下のような出力が得られれば設定が完了しています。

.. code-block:: none

    header:
    seq: 145
    stamp:
        secs: 16
        nsecs: 276000000
    frame_id: "sonar_model::link"
    radiation_type: 0
    field_of_view: 0.0
    min_range: 0.00999999977648
    max_range: 5.0
    range: 1.99000000954
    ---
    header:
    seq: 146
    stamp:
        secs: 16
        nsecs: 380000000
    frame_id: "sonar_model::link"
    radiation_type: 0
    field_of_view: 0.0
    min_range: 0.00999999977648
    max_range: 5.0
    range: 1.95000004768
    ---
    header:
    seq: 147
    stamp:
        secs: 16
        nsecs: 480000000
    frame_id: "sonar_model::link"
    radiation_type: 0
    field_of_view: 0.0
    min_range: 0.00999999977648
    max_range: 5.0
    range: 1.94000005722
    ---