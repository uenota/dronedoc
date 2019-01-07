:tocdepth: 1

GPSを用いた自律飛行
***********************************************************************
このページの内容は `Setup and Configuration of the Navigation Stack on a Robot <http://wiki.ros.org/navigation/Tutorials/RobotSetup>`_ を参考にしています。

以下の赤枠内の部分が経路計画と障害物回避に必要な部分です。
経路計画と障害物回避を含む自律飛行を行うためには、

1. ロボットのベースフレーム( ``base_link`` など)と使用するセンサのフレームの間のTFがブロードキャストされていること
2. センサデータがパブリッシュされていること
3. オドメトリ情報がTFとnav_msgs/Odometryメッセージでパブリッシュされていること
4. geometry_msgs/Twistメッセージを使ってロボットを操作できること

が必要です。
以降ではそれぞれの設定について見ていきます。

.. figure:: imgs/overview_tf_gps.png

  From `ROS Wiki <http://wiki.ros.org/navigation/Tutorials/RobotSetup>`_ (`CC BY 3.0 <https://creativecommons.org/licenses/by/3.0/>`_)

TF
=====================================
今回はLiDARを使用するので、LiDARとロボットの位置関係がわかるように、ロボットのベースフレームからLiDARのフレームへのTFを定義してあげる必要があります。
今回はロボットのベースフレームは ``base_link`` 、LiDARのフレームは ``lidar_link`` となっています。
自分のロボットの設定をモデルなどを参照して確認して、必要な場合は適宜変更してください。

静的なTFをパブリッシュするにはtfパッケージの `static_transform_publisher <http://wiki.ros.org/tf#static_transform_publisher>`_ を使います。
そのために、mymodel_sitl.launchをコピーして新しくmymodel_sitl_tf.launchというファイルを作って、以下の内容を追加してください。

.. code-block:: xml

  <node pkg="tf" name="base2lidar" type="static_transform_publisher" args="0 0 0.1 0 0 0 base_link lidar_link 100"/>


センサ情報
=====================================
今回は障害物の検出に2D LiDARを使います。
``models/iris_2d_lidar/model.sdf`` 内でLiDARの設定を以下のようにしたので、
LiDARの点群データは ``/laser/scan`` トピックにパブリッシュされます。

.. code-block:: xml

  <plugin name="LaserPlugin" filename="libgazebo_ros_laser.so">
    <topicName>/laser/scan</topicName>
    <frameName>lidar_link</frameName>
  </plugin>

また、LiDARの設置してあるフレームは ``lidar_link`` です。
このフレームは ``base_link`` フレームの子要素になっています。
つまり、 ``base_link`` → ``lidar_link`` のようにTFがパブリッシュされています。

オドメトリ情報
=====================================
TFをパブリッシュする
-------------------------------------
move_baseでは、障害物回避と経路計画のためにグローバルとローカルの２つのコストマップを使っています。
今回の設定ではローカルコストマップは ``odom`` フレームを参照し、グローバルコストマップは ``map`` フレームを参照するようになっているので、それぞれのフレームからの ``base_link`` へのTFを定義する必要があります。

``odom`` フレームは一般的にロボットのローカルフレームとして使用され、 ``map`` フレームは一般的にグローバルフレームとして使用されます。
今回もその慣習に従って、ローカルフレームのTFを ``odom`` フレーム内で、グローバルフレームのTFを ``map`` フレーム内でブロードキャストすることにします。

フレームについての詳細は `REP105 -- Coordinate Frames for Mobile Platforms <http://www.ros.org/reps/rep-0105.html>`_ に記述されています。

.. figure:: imgs/coordsystems_img.png

From `ROS Wiki <http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot>`_ (`CC BY 3.0 <https://creativecommons.org/licenses/by/3.0/>`_)

odom->base_link
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
機体のTFはmavrosを使ってパブリッシュすることができます。
しかし、mavros_posix_sitl.launchを使った場合は、デフォルトではTFがパブリッシュされません。

TFがパブリッシュされるようにするには、 ``/mavros/local_position/tf/send`` パラメータの値を ``true`` にする必要があります。

また、以降でmove_baseを使うための設定として、 ``/mavros/local_position/tf/frame_id`` と、 ``/mavros/local_position/tf/frame_id`` のパラメータの値を ``odom`` にしておきます。

以下の内容をmymodel_sitl_tf.launchの最後に追加してください。

.. code-block:: xml

  <param name="/mavros/local_position/tf/send" type="bool" value="true" />
  <param name="/mavros/local_position/frame_id" type="str" value="odom" />
  <param name="/mavros/local_position/tf/frame_id" type="str" value="odom" />

map->odom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
本来、 ``odom`` フレームからのTFはホイールオドメトリなどを用いて計算し、 ``map`` フレームからのTFはGPSやLiDARなどのセンサ情報を用いて更新されるのですが、今回はどちらもGPSを用いている上に基点も同じなので、次のようにして ``map`` から ``odom`` へのTFをブロードキャストすることにします。
以下の内容をmymodel_sitl_tf.launchに追加してください。

.. code-block:: xml

  <node pkg="tf" name="map2odom" type="static_transform_publisher" args="0 0 0 0 0 0 map odom 100"/>


最終的には、 ``map`` から ``lidar_link`` までのTFは下図のようになります。

.. figure:: imgs/frames.png

今回は ``odom`` フレームと ``map`` フレームについて解説するために ``map`` から ``odom`` へのTFを無理やりブロードキャストしていますが、どちらかのフレームしか利用できない場合は、片方のフレームだけでも問題ありません。

nav_msgs/Odometryをパブリッシュする
-------------------------------------
nav_msgs/Odometryメッセージがパブリッシュされるトピックはmavrosには用意されていないので、自分で用意する必要があります。

`ROS Wikiのチュートリアル <http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom>`_ を参考にしてノードを作成しましょう。

``map`` から ``base_link`` へのTFはすでにパブリッシュされているので、今回は ``nav_msgs::Odometry`` メッセージのパブリッシャを書くだけで構いません。

Pythonのコード例は、:doc:`odom_pub_py` を参考にしてください。

.. literalinclude:: ../../src/odom_publisher.cpp
  :language: cpp
  :linenos:
  :caption: odom_publisher.cpp

コード解説
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: cpp

  #include <ros/ros.h>
  #include <nav_msgs/Odometry.h>
  #include <geometry_msgs/PoseStamped.h>
  #include <geometry_msgs/TwistStamped.h>
  #include <geometry_msgs/Quaternion.h>
  #include <tf/transform_datatypes.h>

使用するメッセージを使うのに必要なヘッダファイルをインクルードしています。
``tf/transform_datatypes.h`` は、 ``createQuaternionFromYaw`` 関数を使用するために必要です。

.. code-block:: cpp

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

``nav_msgs/Odometry`` メッセージのパブリッシャです。

.. code-block:: cpp

  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 50, local_pos_cb);
  ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 50, local_vel_cb);

``nav_msgs/Odometry`` メッセージのフィールドで使われる位置と速度のサブスクライバです。

.. code-block:: cpp

  geometry_msgs::PoseStamped local_pos;
  void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    local_pos = *msg;
  }

  geometry_msgs::TwistStamped local_vel;
  void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    local_vel = *msg;
  }

それぞれのサブスクライバのコールバック関数です。
グローバル変数にメッセージを格納するだけのシンプルなものです。

.. code-block:: cpp

    tf::Quaternion tf_quat = tf::createQuaternionFromYaw(local_pos.pose.orientation.z);

    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(tf_quat, odom_quat);

z軸周りの回転（Yaw）からTFのクォータニオンを生成して、それをROSメッセージに変換しています。

.. code-block:: cpp

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = local_pos.pose.position.x;
  odom.pose.pose.position.y = local_pos.pose.position.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = local_vel.twist.linear.x;
  odom.twist.twist.linear.y = local_vel.twist.linear.y;
  odom.twist.twist.angular.z = local_vel.twist.angular.z;

  odom_pub.publish(odom);

``nav_msgs/Odometry`` メッセージの各フィールドを埋めてパブリッシュしています。
``nav_msgs/Odometry`` メッセージのフィールドについては、 `nav_msgs/Odometry Message <http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html>`_ を見てください。

速度指令
=====================================
cmd_velトピック
-------------------------------------
mavrosを使えば、 ``/mavros/setpoint_velocity/cmd_vel_unstamped`` トピックもしくは、 ``/mavros/setpoint_velocity/cmd_vel`` トピックに速度指令を送信することでドローンを移動させることができます。
これらのトピックはメッセージの型によって分けられており、 ``cmd_vel_unstamped`` は ``Twist`` 型、 ``cmd_vel`` は ``TwistStamped`` 型のメッセージを使います。
move_baseは ``Twist`` 型のメッセージをパブリッシュするので、今回は ``/mavros/setpoint_velocity/cmd_vel_unstamped`` トピックを利用します。

また、move_baseは ``/cmd_vel`` トピックに速度指令をパブリッシュするようになっているので、Launchファイル内で次のように `リマップ <http://wiki.ros.org/roslaunch/XML/remap>`_ することでmove_baseからの速度指令がドローンに送られるようにします。

Launchファイルについては、後述の :ref:`navigation_launch` の項を参照してください。

.. code-block:: xml

  <remap from="/cmd_vel" to="/mavros/setpoint_velocity/cmd_vel_unstamped" />

mav_frameパラメータ
-------------------------------------
``cmd_vel`` トピックもしくは、 ``cmd_vel_unstamped`` トピックを使う場合の速度指令は、デフォルトではドローンのベースフレームではなく、ドローンの位置の基準となるフレーム（ ``odom`` や ``map`` など）を基準とした速度を使用しています。

以下の画像のように、速度指令の基準となるフレームが異なる場合には、同じ値の速度指令値を入力しても、ロボットの速度（太い矢印）は異なります。

.. image:: imgs/mav_frame.png

move_baseがパブリッシュする速度指令は、ロボットのベースフレームを基準としたものです。
そのため、今回は速度指令の基準となるフレームをロボットのベースフレームに設定しなければなりません。

``cmd_vel`` トピックによる速度指令が基準とするフレームを変更するには、 ``/mavros/setpoint_velocity/mav_frame`` パラメータの設定を変える必要があります。
ドローンのベースフレームを基準にする場合は、このパラメータの値を ``BODY_NED`` に変更します。
デフォルトでは ``LOCAL_NED`` になっています。

以下の内容をmymodel_sitl_tf.launchに追加することで、このパラメータの値を変更できます。

.. code-block:: xml

  <param name="/mavros/setpoint_velocity/mav_frame"  type="str" value="BODY_NED" />

他の利用可能なフレームについては `mavros/Enumerations <http://wiki.ros.org/mavros/Enumerations>`_ にリストがあります。

設定ファイルを書く
=====================================
大域的経路計画と局所的経路計画
-------------------------------------
コストマップは事前に与えられた地図情報やセンサから得られた障害物の情報をもとに構築される地図で、経路計画に用いられます。（ `参考 <https://qiita.com/MoriKen/items/d5cd6208143d6c40caff#%E5%A4%A7%E5%9F%9F%E3%81%A8%E5%B1%80%E6%89%80%E3%81%AE%E6%AF%94%E8%BC%83>`_ ）

大域的経路計画では、スタートからゴールまでの経路を計画します。
大域的経路計画においては、グローバル（大域的）コストマップが使われます。

局所的経路計画では、障害物回避のための経路を計画します。
局所的経路計画を行うために、ローカル（局所的）コストマップが使用されます。

大域的経路計画を行う経路計画手法のことをグローバルプランナー、局所的経路計画を行う手法のことをローカルプランナーといいます。
グローバルプランナーにはダイクストラ法やA*法が、ローカルプランナーにはDynamic Window Approach（DWA）などがあります。

コストマップの設定
-------------------------------------
コストマップの設定はyamlファイルに記述します。
共通の設定、大域的コストマップの設定、局所的コストマップの設定用の3つの設定ファイルを以下のように準備します。

コストマップのパラメータの設定はグローバルプランナーやローカルプランナーが生成する経路にも影響を与えます。
生成されるグローバルパスやローカルパスが満足の行くものでない場合には、コストマップのパラメータを変更することも検討するとよいかもしれません。

以下の設定は必要最小限のものです。
他の設定やコストマップの詳細については `costmap_2dのROS Wikiページ <http://wiki.ros.org/costmap_2d>`_ を見てください。

共通設定
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. literalinclude:: ../../config/costmap_common_params.yaml
  :language: yaml
  :caption: costmap_common_params.yaml
  :linenos:

それぞれのパラメータの意味は以下のとおりです。

``obstacle_range``
  コストマップに反映する障害物の距離。
  この例では3.0 mより近くにある障害物がコストマップに反映されます。
``raytrace_range``
  センサデータがこの数値以上の場合には、ロボットからこの数値の距離までの間に障害物はないと判断します。
``footprint``
  ロボットの形状を多角形で指定します。中心は ``(0.0, 0.0)`` です。
  ロボットの外形が円である場合には、 ``robot_radius`` パラメータを使います。
``inflation_radius``
  このパラメータで指定された値の距離よりも障害物に近いグリッドには、コストが付与されます。
``observation_sources``
  コストマップに渡される情報を得るセンサを設定します。
``sensor_frame``
  センサーの座標系（フレーム）です。
``data_type``
  トピックにパブリッシュされているメッセージの型（今回は ``sensor_msgs/LaserScan`` ）です。
``topic``
  センサデータがパブリッシュされているトピックです。
``marking``
  ``true`` の場合には、障害物をコストマップに追加する際にこのセンサの情報が使用されます。
``clearing``
  ``true`` の場合には、障害物をコストマップから除去する際にこのセンサの情報が使用されます。

大域的コストマップの設定
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. literalinclude:: ../../config/global_costmap_params.yaml
  :language: yaml
  :caption: global_costmap_params.yaml
  :linenos:

``global_frame``
  大域的コストマップの座標系（フレーム）を設定します。先述したように、 ``odom`` フレームからのTFしか得られない場合はここを ``odom`` にしても構いません。
``robot_base_frame``
  （コストマップが参照する）ロボットのベース座標系（フレーム）を設定します
``update_frequency``
  コストマップの更新周期（Hz）を定義します
``static_map``
  ``true`` にすると、既存のマップもしくはmap_severから提供されるマップを使ってコストマップを初期化されます。
  マップを使わずに初期化する際には ``false`` にします。
``rolling_window``
  ``true`` にすると、ロボットが移動しても局所的コストマップの中心がロボットに追従するようになります。 ``static_map`` パラメータをfalseにした際には ``true`` にする必要があります。
``width``
  コストマップの幅（m）
``height``
  コストマップの高さ（m）
``resolution``
  コストマップの解像度（cell/m）

局所的コストマップの設定
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. literalinclude:: ../../config/local_costmap_params.yaml
  :language: yaml
  :caption: local_costmap_params.yaml
  :linenos:

ほとんどの内容は大域的コストマップの設定と同じです。

``publish_frequency``
  コストマップを表示するためのデータをパブリッシュする周波数（Hz）です

ローカルプランナーの設定
-------------------------------------
ロボットのベースに与える速度指令を計算するためにローカルプランナーを使用します。
パラメータの詳細やローカルプランナーの概要については `base_local_plannerのROS Wikiページ <http://wiki.ros.org/base_local_planner>`_ を見てください。

以下の設定ファイルではロボットの速度と加速度の最大値、最小値等を定義しています。

``holonomic_robot`` パラメータはロボットがホロノミックかどうかを定義します。
ロボットの制御可能な自由度が全体の自由度と等しい場合にはホロノミックであるといい、そうでない場合はノンホロノミック（もしくは非ホロノミック）であるといいます。
例えば、自動車は全体の自由度が3（x, y, yaw）ですが、制御可能な自由度は2（x, yaw）であり、ノンホロノミックなシステムであるといえます。
ドローンの場合は制御可能な自由度（x, y, z, roll, pitch, yaw）が全体の自由度と等しいのでホロノミックなシステムです。

.. literalinclude:: ../../config/base_local_planner_params.yaml
  :language: yaml
  :caption: base_local_planner_params.yaml
  :linenos:

ドローンのパラメータの変更
=====================================
二次元の自己位置推定やマッピングの精度を高めるためには、できるだけセンサを水平に保つ必要があります。
ドローンのパラメータを設定してピッチ角とロール角が一定以上にならないようにしましょう。

また、デフォルトではGCS（Ground Control Station）及びRCとの接続が切れた場合と、OFFBOARDコントロールが切れた場合に自動でホームポジションに戻るようになっているので、フェイルセーフを無効化しておきます。
実機でフェイルセーフを解除する場合には十分に注意して行いましょう。

以下のパラメータの設定を変更します。
利用可能なパラメータ一覧は `Parameter Reference <https://docs.px4.io/en/advanced_config/parameter_reference.html>`_ を参照してください。

.. csv-table::
  :header: パラメータ名, min > max (Incr.), デフォルト, 単位

  FW_P_LIM_MAX, 0.0 > 60.0 (0.5), 45.0, deg
  FW_P_LIM_MIN, -60.0 > 0.0 (0.5), -45.0, deg
  FW_R_LIM, 35.0 > 65.0 (0.5), 50.0, deg
  NAV_DLL_ACT, 0 > 6, 0,
  NAV_RCL_ACT, 0 > 6, 2,
  COM_OBL_ACT, 0 > 2, 0,

パラメータを設定する方法は以下の2種類があります。

- コマンドで設定する
- 設定ファイルを使う

コマンドで設定する方法は簡単ですが、毎回設定しなければなりません。
設定ファイルを使う方法は準備が必要ですが、基本的に一度準備すれば毎回設定し直す必要はありません。

コマンドで設定する
-------------------------------------
PX4シミュレータが起動したら、以下のコマンドを実行します。
今回は試しに以下の数値を設定してみます。
数値は適宜変更してください。

.. code-block:: none

  param set FW_P_LIM_MAX 10.0
  param set FW_P_LIM_MIN -10.0
  param set FW_R_LIM 40.0
  param set NAV_DLL_ACT 0
  param set NAV_RCL_ACT 0
  param set COM_OBL_ACT 0

コマンドが成功したら次のように表示されます。
以下はFW_R_LIMの例です。

.. code-block:: none

  FW_R_LIM: curr: 50.0000 -> new: 40.0000

パラメータの設定を行うコマンドの記法は以下のとおりです。

.. code-block:: none

  param set パラメータ名 値

設定ファイルを使う
-------------------------------------
PX4シミュレーションの起動は、起動スクリプト（ ``~/.ros/etc/init.d-posix/rcS`` ）によって行われ、その中で設定スクリプトが読み込まれることで機体のパラメータなどが設定されます。

シミュレーションで使われる機体の設定スクリプトは、 ``~/.ros/etc/init.d-posix`` 以下にあり、ここからロードされます。
なので、設定ファイルを作成した後にこのディレクトリに移動、もしくはシンボリックリンクを貼る必要があります。
手動でファイルを移動させても良いのですが、更新したあとに移動し忘れたり、初期設定の際に移動し忘れたりする恐れがあります。
そこで、今回はCMakeLists.txtを編集することでビルドしたら自動で移動してくれるようにします。

この節では設定スクリプトの作成とCMakeLists.txtへの記述の追加を行います。

以下のような内容の設定スクリプトを ``px4_sim_pkg/posix_airframes`` 以下に、 ``70010_iris_2d_lidar`` という名前で保存します。
説明されていないパラメータについてはパラメータ一覧を見てください。
設定スクリプトについては、 `Adding a New Airframe Configuration - PX4 Developer Guide <https://dev.px4.io/en/airframes/adding_a_new_frame.html>`_ に説明があります。

.. literalinclude:: ../../posix_airframes/70010_iris_2d_lidar
  :linenos:
  :caption: 70010_iris_2d_lidar

また、以下の内容をCMakeLists.txtに追加します。
以下では、 ``add_custom_target`` を使って、 ``iris_2d_lidar`` という、シンボリックリンクを作成するターゲットを作成しています。

PX4 Firmwareがインストールされていないとエラーが出るので、PX4 Firmwareがインストールされていて、シミュレーションが起動することを確認してからビルドしてください。

.. code-block:: cmake
  :linenos:

  set(PACK_AIRFRAME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/posix_airframes)
  set(LOCAL_AIRFRAME_DIR $ENV{HOME}/.ros/etc/init.d-posix)
  add_custom_target(iris_2d_lidar
                    ALL ln -s -b ${PACK_AIRFRAME_DIR}/70010_iris_2d_lidar ${LOCAL_AIRFRAME_DIR}/)

また、mymodel_sitl.launchの以下の部分を

.. code-block:: xml

  <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_2d_lidar/model.sdf" />

  <include file="$(find px4)/launch/mavros_posix_sitl.launch" >
      <arg name="sdf" value="$(arg sdf)" />
  </include>

次のように変更してください。

.. code-block:: xml

  <arg name="vehicle" default="iris_2d_lidar"/>
  <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_2d_lidar/model.sdf" />

  <include file="$(find px4)/launch/mavros_posix_sitl.launch" >
      <arg name="sdf" value="$(arg sdf)" />
      <arg name="vehicle" value="$(arg vehicle)" />
  </include>

``vehicle`` に指定された機体名と一致する設定スクリプトがPX4 SITLによって読み込まれます。


Launchファイルを書く
=====================================

.. _navigation_launch:

navigation.launch
-------------------------------------
このLaunchファイルでは、コストマップ等の設定ファイルをすべて ``px4_sim_pkg/config`` 以下に保存していることを想定しています。

以下の内容を、 ``px4_sim_pkg/launch`` 以下に、navigation.launchとして保存してください。

.. code-block:: xml
  :caption: navigation.launch
  :linenos:

  <launch>

    <node pkg="px4_sim_pkg" type="odom_publisher" name="odom_publisher"/>

    <remap from="/cmd_vel" to="/mavros/setpoint_velocity/cmd_vel_unstamped"/>

    <node pkg="move_base" type="move_base" name="move_base" respawn="false" output="screen">

      <!-- update frequency of global plan -->
      <param name="planner_frequency" value="2.0"/>

      <!-- Common params for costmap -->
      <rosparam command="load" ns="global_costmap" file="$(find px4_sim_pkg)/config/costmap_common_params.yaml"/>
      <rosparam command="load" ns="local_costmap" file="$(find px4_sim_pkg)/config/costmap_common_params.yaml"/>

      <!-- Params for global costmap -->
      <rosparam command="load" file="$(find px4_sim_pkg)/config/global_costmap_params.yaml"/>

      <!-- Params for local costmap -->
      <rosparam command="load" file="$(find px4_sim_pkg)/config/local_costmap_params.yaml"/>

      <!-- Params for local planner -->
      <rosparam command="load" file="$(find px4_sim_pkg)/config/base_local_planner_params.yaml"/>
    </node>

  </launch>

mymodel_sitl_tf.launch
------------------------------------------
最終的な mymodel_sitl_tf.launchは以下のようになります。

.. code-block:: xml
  :linenos:

  <launch>

      <node pkg="tf" name="base2lidar" type="static_transform_publisher" args="0 0 0.1 0 0 0 base_link lidar_link 100"/>
      <node pkg="tf" name="map2odom" type="static_transform_publisher" args="0 0 0 0 0 0 map odom 100"/>

      <arg name="vehicle" default="iris_2d_lidar"/>
      <arg name="sdf" default="$(find px4_sim_pkg)/models/iris_2d_lidar/model.sdf" />

      <include file="$(find px4)/launch/mavros_posix_sitl.launch" >
          <arg name="sdf" value="$(arg sdf)" />
          <arg name="vehicle" value="$(arg vehicle)" />
      </include>

      <param name="/mavros/local_position/tf/send" type="bool" value="true" />
      <param name="/mavros/local_position/frame_id" type="str" value="odom" />
      <param name="/mavros/local_position/tf/frame_id" type="str" value="odom" />
      <param name="/mavros/setpoint_velocity/mav_frame"  type="str" value="BODY_NED" />

  </launch>

実行してみる
=====================================

ビルドする
-------------------------------------
オドメトリをパブリッシュするためのノードをビルドします。

.. code-block:: bash

  cd ~/catkin_ws
  catkin_make

CMakeLists.txtに以下の内容を追加するのを忘れないようにしましょう。

.. code-block:: cmake

  add_executable(odom_publisher src/odom_publisher.cpp)
  target_link_libraries(odom_publisher ${catkin_LIBRARIES})

Pythonノードを使う場合には忘れずに実行権限を与えておきます。

.. code-block:: bash

  chmod +x odom_publisher.py

ノードを起動する
-------------------------------------
以下のコマンドを用いてLaunchファイルを起動します。

.. code-block:: bash

  roslaunch px4_sim_pkg mymodel_sitl.launch
  roslaunch px4_sim_pkg navigation.launch

.. image:: imgs/gazebomymodel.png

ゴールを送信する
-------------------------------------
ドローンを離陸させる
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ゴールを送信する前にドローンを離陸させます。
シミュレーションのノードを起動しているターミナルで以下のコマンドを実行すると離陸します。

.. code-block:: none

  commander takeoff

ROSサービスを利用しても構いません。

.. code-block:: bash

  rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 47.3977506, longitude: 8.5456074, altitude: 5}"

もしくは、mavrosの提供するノードを使って、以下のように離陸させることもできます。

.. code-block:: bash

  rosrun mavros mavcmd takeoffcur -a 0 0 5

mavrosの提供するサービスやコマンドについては `mavrosのROS Wikiページ <http://wiki.ros.org/mavros>`_ に一覧があります。

Rvizを使ってゴールを送信する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
``move_base/goal`` トピックに ``move_base_msgs/MoveBaseActionGoal`` 型のメッセージを送ってもゴールを設定することができるのですが、Rvizを使ったほうが簡単なので今回はRvizを使用してゴールを送信します。

以下の画像のように、Rvizを使ってmove_baseのゴールを設定することができます。
赤枠内の2D Nav Goalをクリックして選択したあとに、目標位置をクリックして決定します。
クリックしたままマウスカーソルを移動させると目標位置の向きも指定することができます。

.. image:: imgs/rviz_goal.gif

Offboardモードにする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
また、Offboardモードになっていないと ``cmd_vel_unstamped`` トピックを使って操作できないので、Offboardモードにします。
以下のコマンドを使えばドローンの飛行モードがOffboardになります。
``/mavros/setpoint_velocity/cmd_vel_unstamped`` にトピックがパブリッシュされていないとOffboardモードに切り替わらないので、ゴールを指示してからOffboardモードにします。

.. code-block:: bash

  rosservice call /mavros/set_mode "base_mode: 0
  custom_mode: 'OFFBOARD'"

もしくは、先ほどと同様にmavrosのノードを使用しても構いません。

.. code-block:: bash

  rosrun mavros mavsys mode -c OFFBOARD

実行結果
-------------------------------------
上記の手順を踏んで実行した結果が以下の動画です。
ゴールを指定するとゴールに向かって飛行していることがわかります。

.. image:: imgs/nav_gps.gif

この動画内のRvizではTF以外に、以下のトピックにパブリッシュされているメッセージを表示しています。

``/move_base/current_goal``
  現在の目標姿勢
``/move_base/local_costmap/footprint``
  ロボットの外形
``/move_base/local_costmap/costmap``
  局所的コストマップ
``/move_base/TrajectoryPlannerROS/local_plan``
  ローカルパス
``/move_base/TrajectoryPlannerROS/global_plan``
  グローバルパス

これらのメッセージは、"Add"ボタンで表示されるメッセージの追加ウィンドウの、"By Topic"タブからも追加できます。

.. image:: imgs/choose_topic.png

障害物回避
-------------------------------------
ドローンの進路に障害物をおいた場合には迂回する経路が生成されます。

.. image:: imgs/obstacle.gif

この例では更に以下のメッセージをRvizで可視化しています。

``/move_base/global_costmap/costmap``
  大域的コストマップ
``/laser/scan``
  LiDARの計測データ

この動画では、コストマップのカラースキームを、"map"から、"costmap"に変更してあります。

.. image:: imgs/color_scheme.png

経路の障害物からの距離や、コストマップのサイズ、解像度などはコンフィグファイル内のパラメータを変更することで調節できます。
また、他のパラメータについては、costmap_2dパッケージや、base_local_plannerパッケージのWikiページを参照してください。

まとめ
=====================================
お疲れ様でした。
以上までで、ドローンに目標位置を指定して、障害物を考慮した経路計画を実行し、経路上を移動して目標位置まで移動させることができました。

以下のページを参考にしてパラメータをチューニングしたり、別のローカルプランナーを使用してナビゲーションの性能を向上させてもよいでしょう。

- `costmap_2d <http://wiki.ros.org/costmap_2d>`_
- `base_local_planner <http://wiki.ros.org/base_local_planner>`_
- `global_planner <wiki.ros.org/global_planner>`_

別のローカルプランナー使う方法については、 :doc:`../change_local_planner/change_local_planner` を参考にしてください。

参考
=====================================
`move_base - ROS Wiki <http://wiki.ros.org/move_base>`_
  move_baseパッケージ
`Publishing Odometry Information over ROS <http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom>`_
  オドメトリをパブリッシュするノードを書く（ROS Wiki）
`ROS Navigation Stack について2 ~ Odometry生成ノードの作成 ~ <http://daily-tech.hatenablog.com/entry/2017/02/11/182916>`_
  オドメトリをパブリッシュするノードを書く
`How to listen to tf inside a callback loop? <https://answers.ros.org/question/105171/how-to-listen-to-tf-inside-a-callback-loop/>`_
  コールバック関数内でTFをルックアップする方法
`obstacle_range & raytrace_range - precise explanation? <https://answers.ros.org/question/72265/obstacle_range-raytrace_range-precise-explanation/>`_
  obstacle_rangeとraytrace_rangeの意味について
`costmap_2d - ROS Wiki <http://wiki.ros.org/costmap_2d#Map_Types>`_
  static_mapパラメータとrolling_windowパラメータについて