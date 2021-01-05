:tocdepth: 1

自作ノードを実行する（C++）
****************************************************************

この章ではシミュレーション上のドローンを操作するROSノードをC++で作成します。

ROSパッケージを作る
================================================================
はじめに、自作ノードを格納するためのROSパッケージを作ります。
パッケージ名はpx4_sim_pkgとします。
また、roscpp、geometry_msgs、mavros_msgsパッケージを使用するので依存パッケージに追加しておきます。

.. code-block:: bash

    cd ~/catkin_ws/src
    catkin_create_pkg px4_sim_pkg roscpp geometry_msgs mavros_msgs

ノードを書く
================================================================
それでは実際にノードを書いてみましょう。
今回は、ドローンを2m上空に離陸させるノードを書きます。
以下のコードは、 `MAVROS Offboard control example - PX4 Developer Guide <https://dev.px4.io/en/ros/mavros_offboard.html>`_ にあるのと同じものです。

このコードを ``offboard_sample.cpp`` という名前で、 ``px4_sim_pkg/src`` に保存してください。

.. literalinclude:: ../../src/offboard_sample.cpp
    :language: cpp
    :linenos:
    :caption: From `PX4 Developer Guide <https://dev.px4.io/en/ros/mavros_offboard.html>`_ (`CC BY 4.0 <https://creativecommons.org/licenses/by/4.0/deed>`_)

コードの解説
----------------------------------------------------------------

.. code-block:: cpp

    #include <ros/ros.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <mavros_msgs/CommandBool.h>
    #include <mavros_msgs/SetMode.h>
    #include <mavros_msgs/State.h>

mavrosパッケージで提供されるトピックやサービスを使うためのメッセージは ``mavros_msgs`` パッケージで定義されています。

.. code-block:: cpp

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

ROSノードを ``offb_node`` という名前で初期化しています。

.. code-block:: cpp

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

``mavros/state`` トピックにパブリッシュされる、 ``mavros_msgs::State`` 型のメッセージをサブスクライブするためのサブスクライバを初期化しています。
第二引数は受け取ったメッセージを保持しておくキューのサイズです。
第三引数の ``state_cb`` は受け取ったメッセージを処理するためのコールバック関数です。
``state_cb`` は次のように定義されています。

``mavros_msgs::State`` 型のメッセージには、機体の接続状況やモータの動作状況などが格納されています。

.. code-block:: cpp

    mavros_msgs::State current_state;
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

``state_cb`` は ``const mavros_msgs::State::ConstPtr&`` 型の変数を受け取り、それを ``mavros_msgs::State`` 型のグローバル変数 ``current_state`` に格納する関数です。
コールバック関数では、 ``パッケージ名::メッセージ型::ConstPtr&`` のような型の変数を受け取るようにするのが一般的です。
細かいことはC++の本で勉強しましょう。

.. code-block:: cpp

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

``mavros/setpoint_position/local`` トピックに ``geometry_msgs::PoseStamped`` 型のメッセージをパブリッシュするパブリッシャを初期化しています。
第二引数は受け取ったメッセージを保持しておくキューのサイズです。

.. code-block:: cpp

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

モータの起動に用いる ``mavros/cmd/arming`` と モードの変更に用いる ``mavros/set_mode`` サービスのクライアントを初期化しています。
それぞれ、 ``mavros_msgs::CommandBool`` 型と ``mavros_msgs::SetMode`` 型のメッセージを使います。

.. code-block:: cpp

    ros::Rate rate(20.0);

Offboardコマンドのタイムアウト時間は500msなので、2Hz以上の周波数で機体にコマンドを送る必要があります。
また、Offboardコマンドがタイムアウトした場合は、その直前の飛行モードに切り替わるので、PositionモードからOffboardモードに切り替えるようにしておけば、Offboardモードへの移行が失敗してもその場にとどまってくれます。

.. code-block:: cpp

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

FCUと接続するまで待ちます。
``current_state.connected`` はFCUと接続されているときに ``true`` になるので、接続されるとwhileループから抜けます。
``ros::spinOnce()`` を使ってループ中でもコールバック関数がきちんと呼ばれるようにします。
また、 ``rate.sleep()`` は、このノードが20Hzで動作するように必要な時間だけノードを停止します。

.. code-block:: cpp

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

あらかじめ移動先の位置をパブリッシュしておきます。
移動先の位置をパブリッシュしておかないとOffboardモードへの移行がリジェクトされます。
ここでは、移動先の位置は上方向に2mとなっています。
``geometry_msgs::PoseStamped`` の各フィールドについては `geometry_msgs/PoseStamped Message <http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html>`_ を見てください。

.. code-block:: cpp

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

``SetMode`` メッセージの ``custom_mode`` フィールドを ``OFFBOARD`` にします。
このメッセージをサービスサーバに送信することでモードの切り替えをおこないます。
``mavros_msgs::SetMode`` メッセージの定義は `mavros_msgs/SetMode Service <http://docs.ros.org/api/mavros_msgs/html/srv/SetMode.html>`_ から見れます。

.. code-block:: cpp

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

``/mavros/cmd/arming`` サービスへ送信するためのメッセージを作成します。
``mavros_msgs::CommandBool`` メッセージの定義は `mavros_msgs/CommandBool Service <http://docs.ros.org/api/mavros_msgs/html/srv/CommandBool.html>`_ にあります。
mavrosパッケージで使用するメッセージの定義は、 `mavrosのROS wikiページ <http://wiki.ros.org/mavros#Plugins>`_ から見ることができます。

.. code-block:: cpp
    :linenos:

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){

            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

最初のif節は現在のモードが ``OFFBOARD`` かつ、モード切替が行われたのが5秒以上前であるときに実行されます。
最後のモード切替からの経過時間は ``last_request`` 変数を使って判断しています。

``set_mode_client.call(offb_set_mode)`` で、 ``offb_set_mode`` 変数のメッセージを使って ``mavros/set_mode`` サービスをコールしています。
サービスをコールした場合、サービスコールが成功すると ``true`` が帰ってきて、失敗すると ``false`` が帰ってきます。なので、戻り値を調べることでサービスコールが成功したかを知ることができます。
また、サービスにはリクエストのフィールド（サービスコールの際に必要な情報を与えるために使う）と、レスポンスのフィールド（処理結果等を格納するのに使う）があり、今回はレスポンスの ``mode_sent`` を調べることでモード切り替えの命令がきちんと認識され、送信されたかを判断しています。

モードが ``OFFBOARD`` の時は機体にモータを起動するように命令を送っています。
この部分の処理はOffboardコマンドを送るところとほぼ同じです。

最後に、ループ毎に目標位置をパブリッシュしています。

CMakeLists.txtの編集
-----------------------------------------------------------------
次にCMakeLists.txtの編集をおこないます。
CMakeLists.txtの末尾に以下の行を追加します。

.. code-block:: cmake

    add_executable(offboard_sample src/offboard_sample.cpp)
    target_link_libraries(offboard_sample ${catkin_LIBRARIES})

ビルドする
-----------------------------------------------------------------

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make

ノードの起動
=================================================================
はじめにシミュレータを起動します。
ROSノードとの通信を行うためにはmavrosが必要なので、 ``mavros_posix_sitl.launch`` を使います。

.. code-block:: bash

    roslaunch px4 mavros_posix_sitl.launch

次に先ほど作成したノードを起動します。

.. code-block:: bash

    rosrun px4_sim_pkg offboard_sample

.. image:: imgs/runningnode_cpp.png

launchファイルを書く
================================================================
シミュレータとOffboardノードを別々に起動するのは面倒なので、launchファイルにまとめて一つのコマンドで起動できるようにしましょう。

``px4_sim_pkg/launch`` 以下に ``cpp_offb_sample.launch`` という名前で以下の内容を保存してください。

.. code-block:: xml

    <launch>

        <include file="$(find px4)/launch/mavros_posix_sitl.launch" />
        <node name="offb_node" pkg="px4_sim_pkg" type="offboard_sample" />

    </launch>

以下のコマンドを使ってこのlaunchファイルからシミュレータとOffboardノードを起動できます。

.. code-block:: bash

    roslaunch px4_sim_pkg cpp_offb_sample.launch

参考
================================================================
`Callbacks and Spinning - ROS Wiki <http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning>`_
    ros::spinOnce()について
`mavros - ROS Wiki <http://wiki.ros.org/mavros>`_
    mavrosのROS Wikiページ