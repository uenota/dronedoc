MoveIt!を使ってGazeboモデルを動かす
============================================================================
この記事では、前回までに作成したiris_moveit_configパッケージとMoveIt! RVizプラグインを使って動作計画を行い、ドローンの制御を行います。

ドローンを移動させられるようにするには、プラグインやアクションサーバーなどを実装する必要があるので、はじめにMoveIt!を用いてロボットを動かす方法と、今回使用するアプリケーションの概要を説明します。

MoveIt!を用いてロボットを動かす方法
----------------------------------------------------------------------------
MoveIt!を用いてロボットを動かすには、主に、

1. MoveIt!の提供するコントローラマネージャを使う
2. ros_controlを使う

の２つの方法があります。
以下では、これら２つの方法について説明します。

コントローラマネージャを使う
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:doc:`../run_demo/run_demo` では、RVizを用いてドローンの動作計画を行いました。
動作計画を行うと、生成されたドローンの動作がRVizに表示されましたが、RVizで表示されるのは計画だけなので実際にロボットを動かすには制御入力などをロボットに送るコントローラーが必要になります。

ロボットを制御するのに必要な制御則や入力は、ロボットの種類によって異なるので、違うロボットには違うコントローラを使わなければなりません。
しかし、異なるロボットのコントローラを全てMoveIt!の開発者が実装したり、誰かが実装したコントローラをその都度MoveIt!に取り入れるのは現実的ではありません。
そこで、MoveIt!では、`ROSアクション <http://wiki.ros.org/actionlib>`_ の仕組みを用いることで、MoveIt!の提供するアクションに対応するコントローラであればどのようなものでも使用できるようにしてあります。

ROSアクションは、ROSサービスに似た、ノードが一対一で通信するための仕組みです。
サービスは、クライアントがサーバにリクエストを送信すると、結果が帰ってくるまではサーバーからのフィードバックは一切ありませんが、アクションでは実行状況に応じてフィードバックが帰ってきます。
また、アクションは途中で中断することも可能で、ロボットをスタートからゴールに移動させるような、時間のかかる処理に適しています。

MoveIt!では、下図のように、コントローラインターフェース（以下インターフェース）内で定義されたアクションクライアントが、アクションの目的であるアクションゴールを、コントローラ内で定義されたアクションサーバに送信し、それを受け取ったアクションサーバがロボットへ制御指令を入力するような形をとっています。
使用するアクションの種類によって、用いるインターフェースが異なります。

下図中のコントローラマネージャ（以下マネージャ）は、インターフェースをMoveIt!に登録する役割があります。
MoveIt!にインターフェースを登録することで、生成された動作を実行する命令を受けた際に、動作の情報がインターフェースへと送られ、そこからコントローラへと送信されるようになります。
マネージャにはいくつかの種類があり、種類によって使用できるインターフェースが異なります。

.. image:: imgs/moveit_manager.png

生成された動作を実行する際の流れを図示すると以下のようになります。

1. インターフェースに実行する動作の情報が送信される
2. インターフェースがアクションサーバに対してアクションゴール（目標）を送信する
3. アクションゴールを受け取ったコントローラがロボットに制御入力を送る

.. image:: imgs/moveit_flow.png

以上をまとめると、MoveIt!では、インターフェースが実行する動作の情報をコントローラへ送信し、それを受け取ったコントローラが、ロボットを動作させるための制御入力をロボットに送信する、といった形になっています。
インターフェースが、実行される動作の情報をコントローラへ送信できるようにするためには、マネージャがインターフェースをMoveIt!に登録する必要があります。

以下では、マネージャ、インターフェース、コントローラのそれぞれの要素について更に詳しく見ていきます。

マネージャ
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MoveIt!がデフォルトで提供しているマネージャには、以下の２つのものがあります。

- MoveItFakeControllerManager
- MoveItSimpleControllerManager

MoveItFakeControllerManagerは、シミュレーションで用いられるマネージャで、RVizを使って動作を可視化するときなどに使われています。
MoveItSimpleControllerManagerは、FollowJointTrajectoryとGripperCommandの２つのインターフェースをサポートしており、これらは実際にロボットを制御する際に使うことができます。

マネージャは ``controller_list`` パラメータに指定されたインターフェースをMoveIt!に登録します。
同時に、制御される関節などの情報も与えられ、これをもとにロボットの制御が行われます。
パラメータに関しては、`Low Level Controllers <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/controller_configuration/controller_configuration_tutorial.html>`_ を見てください。

また、使用するマネージャは、 ``moveit_controller_manager`` の値を変更することで指定できます。

インターフェース
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MoveIt!がデフォルトで提供するインターフェースは以下の２つです。

FollowJointTrajectoryインターフェース
    アームもしくはロボット自体の制御
GripperCommandインターフェース
    グリッパーの制御

それぞれ、`FollowJointTrajectory <http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html>`_ アクションと、`GripperCommand <https://www.google.com/search?client=ubuntu&channel=fs&q=gripper+command+action&ie=utf-8&oe=utf-8>`_ アクションを使用します。

コントローラ
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MoveItSimpleControllerManagerを使用する場合には、コントローラは自分で実装する必要があります。

上述の通り、FollowJointTrajectoryインターフェースはFollowJointTrajectoryアクションを、GripperCommandは、GripperCommandアクションを使うので、これらのインターフェースを使う場合には適したアクションサーバーを実装する必要があります。

アクションサーバーの実装に関しては、`actionlibのチュートリアル <http://wiki.ros.org/actionlib/Tutorials>`_ を参照してください。

ros_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros_control <https://github.com/ros-controls/ros_control/wiki>`_ には、関節の速度制御や力制御、位置制御などを行うためのプラグインが複数用意されています。
ros_controlに関しては、以下のリンクが参考になります。

- `Tutorial: ROS Control <http://gazebosim.org/tutorials?tut=ros_control>`_
- `Controller と HardwareInterface との間の処理の仕組み（1. ロボットモデルの定義と登録）- Qiita <https://qiita.com/MoriKen/items/613635b90f3a98042dc5>`_
- `Controller と HardwareInterface との間の処理の仕組み（2. RobotHWSimのプラグインについて）- Qiita <https://qiita.com/MoriKen/items/5cab7436c1b36c25e0ce>`_
- `Controller と HardwareInterface との間の処理の仕組み（3. Controllerについて）- Qiita <https://qiita.com/MoriKen/items/c29f653d03baffe5f0e2>`_
- `実機を動かす時の仕様について · Issue #46 · Nishida-Lab/motoman_project <https://github.com/Nishida-Lab/motoman_project/issues/46>`_

アプリケーションの構成
----------------------------------------------------------------------------
今回使用するアプリケーション全体の構成は以下のようになっています。

.. image:: imgs/moveit_uav.png

上で説明した基本の構成に、動作計画の設定を行うRVizとドローンのシミュレーションを行うPX4 SITLシミュレータが追加されています。
この構成では、コントローラはROSアクションを通じて生成された動作の情報を受け取り、生成した経由点を ``/mavros/setpoint_position/local`` トピックにパブリッシュすることでドローンを制御します。

RViz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RVizは :doc:`../run_demo/run_demo` で見たように、経路の設定をMoveIt!側に送り、MoveIt!が生成した経路の情報を元に可視化を行います。

マネージャ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
MoveItSimpleControllerManagerは、FollowJointTrajectoryインターフェースとGripperCommandインターフェースにしか対応しておらず、FollowJointTrajectoryインターフェースは多自由度の関節の制御には使えないので、新たなマネージャを作成する必要があります。

今回はMoveItMultiDOFControllerManagerという名前の新しいマネージャを作成します。
コードの詳細については、:doc:`ctrl_manager` を参照してください。

.. literalinclude:: ../../../src/moveit_multi_dof_controller_manager.cpp
    :linenos:
    :language: cpp
    :caption: moveit_multi_dof_controller_manager.cpp

インターフェース
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
上述の通り、FollowJointTrajectoryは他自由度の関節の制御には使えないので、他自由度の関節用のFollowMultiDOFJointTrajectoryインターフェースを新しく作成します。

インターフェースはsrcディレクトリではなく、include/<package_name>ディレクトリに保存します。
<package_name>の部分は自分の環境に合わせて変更してください。

コードの詳細については、:doc:`ctrl_interface` を参照してください。

.. literalinclude:: ../../../include/dronedoc/follow_multi_dof_joint_trajectory_controller_handle.hpp
    :linenos:
    :language: cpp
    :caption: follow_multi_dof_joint_trajectory_controller_handle.hpp

コントローラ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
今回は、 ``FollowMultiDOFJointTrajectory`` アクションのGoalを受け取り、それをドローンに目標位置として送信するノードをコントローラーとして用います。

コードの詳細については、:doc:`controller` を参照してください。

.. literalinclude:: ../../../src/drone_controller.cpp
    :linenos:
    :language: cpp
    :caption: drone_controller.cpp

CMakeLists.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
``find_package`` に使用するパッケージを追加します。

.. code-block:: cmake

    find_package(catkin REQUIRED COMPONENTS
        ...
        actionlib
        trajectory_msgs
        pluginlib
        moveit_msgs
        moveit_core
    )

コントローラインターフェースをインクルードできるように、インクルードディレクトリを指定します。

.. code-block:: cmake

    catkin_package(
        INCLUDE_DIRS include
        ...
    )

コントローラマネージャをコンパイルしてライブラリをリンクするために以下の内容を追加します。
コントローラマネージャはノードではなく、ライブラリなので、 ``add_library`` を使います。

他のパッケージのライブラリと名前が重複しないようにライブラリ名の先頭に ``${PROJECT_NAME}_`` をつけます。
今回のプロジェクト名は ``dronedoc`` なので、こうすることで ``libdronedoc_moveit_multi_dof_controller_manager`` という名前のライブラリができます。

.. code-block:: cmake

    add_library(${PROJECT_NAME}_moveit_multi_dof_controller_manager src/moveit_multi_dof_controller_manager.cpp)
    target_link_libraries(${PROJECT_NAME}_moveit_multi_dof_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

コントローラをコンパイルしてライブラリをリンクするために以下の内容を追加します。

.. code-block:: cmake

    add_executable(drone_controller src/drone_controller.cpp)
    target_link_libraries(drone_controller ${catkin_LIBRARIES})

moveit_multi_dof_controller_manager_plugin_description.xml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
プラグインローダーがプラグインを見つけられるように、プラグインの情報を記述するファイルを作成します。

.. literalinclude:: ../../../moveit_multi_dof_controller_manager_plugin_description.xml
    :linenos:
    :language: xml
    :caption: moveit_multi_dof_controller_manager_plugin_description.xml

それぞれのタグ及びフィールドの意味は以下のとおりです。

``path``
    ライブラリのパス。CMakeLists.txtの ``add_library`` 内で指定した名前の先頭に ``lib`` を追加したもの（今回の場合は ``libdronedoc_moveit_multi_dof_controller_manager`` ）がライブラリ名になる。
``type``
    プラグインの型名。名前空間から指定する必要がある。
``base_class``
    プラグインのベースクラスの型名。名前空間から指定する必要がある。
``name``
    プラグイン名
``description``
    プラグインの説明

package.xml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
pluginlibを使用するので、package.xmlの依存パッケージにpluginlibを追加します。

.. code-block:: xml

  <depend>pluginlib</depend>

また、``<export>`` タグ内でプラグインをエクスポートします。
``moveit_core`` の部分は、プラグインのベースクラスが定義されているパッケージ名を指定します。
今回は ``moveit_core`` パッケージで ``MoveItSimpleControllerManager`` が定義されているので、これを指定します。

.. code-block:: xml

  <export>
    <moveit_core plugin="${prefix}/moveit_multi_dof_controller_manager_plugin_description.xml"/>
  </export>

プラグインが登録されているかを確認するには、パッケージをビルドしたあとに、以下のように ``rospack`` コマンドを使用します。

.. code-block:: bash

    rospack plugins --attrib=plugin moveit_core

Launch files
----------------------------------------------------------------------------
今まで構築してきたアプリケーションを実行するためにいくつかのLaunchファイルを作成・編集します。

iris_moveit.launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
iris_moveit.launchは、シミュレーション環境（Gazebo+PX4 SITL+MAVROS）とMoveIt!のノード、コントローラの起動を行います。

.. code-block:: xml
    :linenos:
    :caption: iris_moveit.launch

    <launch>

        <!-- robot description for px4 sitl -->
        <arg name="model_sitl" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro_sitl/urdf/iris_base.xacro" />
        <arg name="mavlink_udp_port" default="14560"/>
        <arg name="rotors_description_dir_sitl" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro_sitl" />
        <arg name="cmd_sitl" default="$(find xacro)/xacro $(arg model_sitl) rotors_description_dir:=$(arg rotors_description_dir_sitl) mavlink_udp_port:=$(arg mavlink_udp_port) --inorder"/>
        <param command="$(arg cmd_sitl)" name="robot_description_sitl" />

        <!-- robot description for moveit -->
        <arg name="model" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro/urdf/iris_base.xacro" />
        <arg name="rotors_description_dir" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro" />
        <arg name="cmd" default="$(find xacro)/xacro $(arg model) rotors_description_dir:=$(arg rotors_description_dir) mavlink_udp_port:=$(arg mavlink_udp_port) --inorder"/>
        <param command="$(arg cmd)" name="robot_description" />

        <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

        <!-- PX4 configs -->
        <arg name="est" default="ekf2"/>
        <arg name="vehicle" default="iris_2d_lidar"/>
        <arg name="interactive" default="true"/>
        <env name="PX4_SIM_MODEL" value="$(arg vehicle)" />
        <env name="PX4_ESTIMATOR" value="$(arg est)" />
        <!-- PX4 SITL and Gazebo -->
        <arg unless="$(arg interactive)" name="px4_command_arg1" value="-d"/>
        <arg     if="$(arg interactive)" name="px4_command_arg1" value=""/>
        <node name="sitl" pkg="px4" type="px4" output="screen"
            args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS $(arg px4_command_arg1)" required="true"/>

        <!-- Launching gazebo -->
        <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>
        <arg name="debug" default="false"/>
        <arg name="verbose" default="false"/>
        <arg name="gui" default="true"/>
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="world_name" value="$(arg world)"/>
            <arg name="verbose" value="$(arg verbose)"/>
            <arg name="gui" value="$(arg gui)"/>
            <arg name="debug" value="$(arg debug)"/>
        </include>

        <!-- MAVROS -->
        <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
        <arg name="gcs_url" default="" />
        <arg name="respawn_mavros" default="false" />
        <include file="$(find mavros)/launch/px4.launch">
            <!-- GCS link is provided by SITL -->
            <arg name="gcs_url" value=""/>
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="respawn_mavros" value="$(arg respawn_mavros)"/>
        </include>
        <!-- mavros -->
        <param name="/mavros/local_position/tf/send" type="bool" value="true" />
        <param name="/mavros/local_position/frame_id" type="str" value="base_link" />
        <param name="/mavros/local_position/tf/frame_id" type="str" value="map" />

        <!-- spawn robot -->
        <arg name="x" default="0"/>
        <arg name="y" default="0"/>
        <arg name="z" default="0"/>
        <arg name="R" default="0"/>
        <arg name="P" default="0"/>
        <arg name="Y" default="0"/>
        <node name="$(anon vehicle_spawn)" pkg="gazebo_ros" type="spawn_model" output="screen" args="-param robot_description_sitl -urdf -model $(arg vehicle) -package_to_model -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/>

        <!-- static tf from map to world which is base frame for planning -->
        <node pkg="tf" name="map2world" type="static_transform_publisher" args="0 0 0 0 0 0 map world 100"/>
        <!-- publish tf to supress warnings -->
        <node pkg="tf" name="map2origin" type="static_transform_publisher" args="0 0 0 0 0 0 map local_origin 100"/>
        <node pkg="tf" name="base2fcu" type="static_transform_publisher" args="0 0 0 0 0 0 base_link fcu 100"/>
        <!-- Moveit -->
        <include file="$(find iris_moveit_config)/launch/move_group.launch" >
            <arg name="fake_execution" value="false" />
        </include>

        <node pkg="px4_sim_pkg" name="drone_controller" type="drone_controller" output="screen"/>

    </launch>

解説
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
PX4 SITLシミュレーションで使用するURDFをxacroファイルから生成します。
:doc:`../create_config_pkg/create_config_pkg` で説明したように、PX4 SITLシミュレーションで使用するxacroファイルとMoveIt!で使用するxacroファイルは同じものを使えないので、それぞれ別にロードします。

:doc:`../create_config_pkg/create_config_pkg` では、multirotor_base.xacroファイル内の、 ``package://rotors_description`` を、全て ``package://mavlink_sitl_gazebo/models/rotors_description/`` に置き換えましたが、xacro_sitlディレクトリに置き換えなかったパターンのものを用意し、それをロードします。

.. code-block:: xml

    <arg name="model_sitl" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro_sitl/urdf/iris_base.xacro" />
    <arg name="mavlink_udp_port" default="14560"/>
    <arg name="rotors_description_dir_sitl" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro_sitl" />
    <arg name="cmd_sitl" default="$(find xacro)/xacro $(arg model_sitl) rotors_description_dir:=$(arg rotors_description_dir_sitl) mavlink_udp_port:=$(arg mavlink_udp_port) --inorder"/>
    <param command="$(arg cmd_sitl)" name="robot_description_sitl" />

MoveIt!用のURDFをロードし、joint_state_publisherノードとrobot_state_publisherノードを使ってロボットの関節やフレームの状態をTFとしてパブリッシュします。

.. code-block:: xml

    <arg name="model" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro/urdf/iris_base.xacro" />
    <arg name="rotors_description_dir" default="$(find px4_sim_pkg)/models/iris_depth_camera/xacro" />
    <arg name="cmd" default="$(find xacro)/xacro $(arg model) rotors_description_dir:=$(arg rotors_description_dir) mavlink_udp_port:=$(arg mavlink_udp_port) --inorder"/>
    <param command="$(arg cmd)" name="robot_description" />

    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

ベースリンクへのTFを送信するように設定します。

.. code-block:: xml

    <param name="/mavros/local_position/tf/send" type="bool" value="true" />
    <param name="/mavros/local_position/frame_id" type="str" value="base_link" />
    <param name="/mavros/local_position/tf/frame_id" type="str" value="map" />

iris_groupの親フレームは ``world`` になっていたので、 ``map`` から ``world`` へのTFを定義して、 ``world`` から ``base_link`` へのTFを利用できるようにします。

.. code-block:: xml

    <node pkg="tf" name="map2world" type="static_transform_publisher" args="0 0 0 0 0 0 map world 100"/>

TFが利用不可能であるという警告が出るので、以下のTFを定義して警告が出ないようにします。
なくても構いません。

.. code-block:: xml

    <node pkg="tf" name="map2origin" type="static_transform_publisher" args="0 0 0 0 0 0 map local_origin 100"/>
    <node pkg="tf" name="base2fcu" type="static_transform_publisher" args="0 0 0 0 0 0 base_link fcu 100"/>

MoveIt!のノードを起動します。
実際のロボットに指令を送信するので、``fake_execution`` を ``false`` にします。

.. code-block:: xml

    <include file="$(find iris_moveit_config)/launch/move_group.launch" >
        <arg name="fake_execution" value="false" />
    </include>


iris_moveit_controller_manager.launch.xml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
iris_moveit_controller_manager.launch.xmlは、コントローラマネージャの指定とパラメータのロードを行います。

``moveit_controller_manager`` パラメータを変更して、作成したコントローラマネージャを使用するようにします。
また、後述のcontrollers.yamlファイルをロードしてコントローラマネージャ用のパラメータをロードします。

.. literalinclude:: ../../../../iris_moveit_config/launch/iris_moveit_controller_manager.launch.xml
    :linenos:
    :language: xml
    :caption: iris_moveit_config/launch/iris_moveit_controller_manager.launch.xml

controller.yaml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
コントローラマネージャが使用するパラメータを記述してあるファイルです。
パラメータに関しては、`Low Level Controllers <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/controller_configuration/controller_configuration_tutorial.html>`_ を見てください。

.. literalinclude:: ../../../../iris_moveit_config/config/controllers.yaml
    :linenos:
    :language: yaml
    :caption: iris_moveit_config/config/controllers.yaml

trajectory_execution.launch.xml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
trajectory_execution.launch.xmlは、動作の実行に関するパラメータを設定しています。

現在位置がスタート位置から離れているとエラーが出て動作の実行ができません。
デフォルトでは1cm離れていたら実行できないので、 ``trajectory_execution/allowed_start_tolerance`` パラメータの数値を変更してある程度離れていても動作の実行ができるようにします。

.. code-block:: xml

    <param name="trajectory_execution/allowed_start_tolerance" value="0.1"/>

また、デフォルトでは動作の実行が始まってからある程度時間が経つと動作の実行が中断されるので、``trajectory_execution/execution_duration_monitoring`` パラメータを ``false`` にすることでタイムアウトしないようにします。

.. code-block:: xml

    <param name="trajectory_execution/execution_duration_monitoring" value="false" />

Execution
----------------------------------------------------------------------------
シミュレーション環境とMoveIt!のノードを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg iris_moveit.launch

経路計画用にMoveIt!のプラグインを含むRVizを起動します。

.. code-block:: bash

    roslaunch iris_moveit_config moveit_rviz.launch config:=true

ドローンを離陸させてから設定を行い、"Planning"タブから"Plan"をクリックすると生成されたパスがRVizに表示されます。
スタートが離陸前の位置になっている場合には、"Planning"タブの"Query"にある、"Select Start State"のプルダウンメニューを"current"にして"Update"ボタンを押して現在の位置をスタートとして設定します。

"Execute"ボタンを押すと、``follow_multi_dof_joint_trajectory`` アクションのサーバーにゴールが送信され、アクションサーバーがmavrosのトピックを通じて目標位置を送信し始めます。
``/mavros/setpoint_position/local`` トピックのメッセージを使ってドローンを制御するために、モードをOFFBOARDに変更します。

.. code-block:: bash

    rosrun mavros mavsys mode -c OFFBOARD

以下のようにドローンが目標位置に移動すれば成功です。

.. image:: imgs/planning.gif

障害物がある場合にはそれを回避する経路を生成してくれます。

.. image:: imgs/collision.gif

現状では経路計画の際にロール角やピッチ角の制限を課していないので、ドローンが取ることのできない姿勢が出力されることがあります。
``setpoint_position/local`` トピックを用いた制御では、ヨー角のみが反映されるのでドローンの姿勢が異常になることは無いのですが、MoveIt!によって生成された動作を忠実に実行できないので、RVizの表示では回避できている障害物に衝突することがあります。
`Move Group Interface <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#planning-with-path-constraints>`_ を使えば経路の制限をすることができるので、これを使ってもいいかもしれません。

まとめ
----------------------------------------------------------------------------
この記事では、MoveIt!を用いて計画した経路の経由点をmavros経由でドローンに送ることでドローンを移動させました。

今回は生成された経路を線形補間を用いて補間し、それを ``setpoint_position`` トピックに与えることでドローンを制御しましたが、`waypoint <http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.waypoint>`_ を用いたドローンの位置制御を試してみてもいいかもしれません。

参考
----------------------------------------------------------------------------
pluginlib関連
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- `pluginlib/Troubleshooting <http://wiki.ros.org/pluginlib/Troubleshooting>`_
- `navigationスタックで学ぶpluginlibの使い方 <http://makemove.hatenablog.com/entry/2015/09/29/001725>`_

Move Group Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
今回はRVizからゴールやプランナの設定等を行いましたが、C++やPythonのコードからこれらの設定を行うためのインターフェースが用意されています。

- `Move Group C++ Interface <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html>`_
- `Move Group Python Interface <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html>`_

マニピュレータの制御
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- `ROSによるマニピュレータの制御 <https://sites.google.com/site/robotlabo/time-tracker/ros/ros-manipulator>`_
- `Gazeboによるマニピュレータのシミュレーション  <https://sites.google.com/site/robotlabo/time-tracker/ros/gazebo_mani>`_