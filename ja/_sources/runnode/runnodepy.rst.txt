:tocdepth: 1

自作ノードを実行する（Python）
****************************************************************

この章ではシミュレーション上のドローンを操作するROSノードをPythonで作成します。

ROSパッケージを作る
================================================================
はじめに、自作ノードを格納するためのROSパッケージを作ります。
パッケージ名はpx4_sim_pkgとします。
また、rospy、geometry_msgs、mavros_msgsパッケージを使用するので依存パッケージに追加しておきます。

.. code-block:: bash

    cd ~/catkin_ws/src
    catkin_create_pkg px4_sim_pkg rospy geometry_msgs mavros_msgs

ノードを書く
================================================================
それでは実際にノードを書いてみましょう。
今回は、ドローンを2m上空に離陸させるノードを書きます。

このコードを ``offboard_sample.py`` という名前で、 ``px4_sim_pkg/script`` に保存してください。

.. literalinclude:: ../../script/offboard_sample.py
    :language: python
    :linenos:
    :caption: offboard_sample.py

コードの解説
----------------------------------------------------------------
.. code-block:: python

    #!/usr/bin/env python

ROSノードとして実行するPythonプログラムには一行目にこの宣言が必要です。
詳細は以下のリンクを参照してください。

- `Python初歩からの学習メモ1 <https://qiita.com/sesame/items/47d2694616eadb35d4bc#python%E3%82%B9%E3%82%AF%E3%83%AA%E3%83%97%E3%83%88%E3%81%AE%E5%AE%9F%E8%A1%8C>`_
- `#!/bin/sh は ただのコメントじゃないよ！ Shebangだよ！ <https://qiita.com/mohira/items/566ca75d704072bcb26f>`_

.. code-block:: python

    import rospy

    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, CommandBoolRequest
    from mavros_msgs.srv import SetMode, SetModeRequest

mavrosパッケージで提供されるトピックやサービスを使うためのメッセージは ``mavros_msgs`` パッケージで定義されています。

.. code-block:: python

    rospy.init_node("offb_node")

ROSノードを ``offb_node`` という名前で初期化しています。

.. code-block:: python

    r = rospy.Rate(20)

Offboardコマンドのタイムアウト時間は500msなので、2Hz以上の周波数で機体にコマンドを送る必要があります。
また、Offboardコマンドがタイムアウトした場合は、その直前の飛行モードに切り替わるので、PositionモードからOffboardモードに切り替えるようにしておけば、Offboardモードへの移行が失敗してもその場にとどまってくれます。

.. code-block:: python

    rospy.Subscriber("mavros/state", State, state_cb)

``mavros/state`` トピックにパブリッシュされる、 ``State`` 型のメッセージをサブスクライブするためのサブスクライバを初期化しています。
第二引数は受け取ったメッセージを保持しておくキューのサイズです。
第三引数の ``state_cb`` は受け取ったメッセージを処理するためのコールバック関数です。
``state_cb`` は次のように定義されています。

``State`` 型のメッセージには、機体の接続状況やモータの動作状況などが格納されています。

.. code-block:: python

    current_state = State()
    def state_cb(msg):
        global current_state
        current_state = msg

``state_cb`` は ``State`` 型の変数を受け取り、それをグローバル変数 ``current_state`` に格納する関数です。
受け取ったメッセージをローカル変数ではなく、グローバル変数に格納するために ``global`` キーワードを使っています。

.. code-block:: python

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",
                                     PoseStamped,
                                     queue_size=1000)

``mavros/setpoint_position/local`` トピックに ``PoseStamped`` 型のメッセージをパブリッシュするパブリッシャを初期化しています。
第三引数は受け取ったメッセージを保持しておくキューのサイズです。

.. code-block:: python

    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

モータの起動に用いる ``mavros/cmd/arming`` と モードの変更に用いる ``mavros/set_mode`` サービスのクライアントを初期化しています。
それぞれ、 ``CommandBool`` 型と ``SetMode`` 型のメッセージを使います。

.. code-block:: python

    while not rospy.is_shutdown() and not current_state.connected:
        r.sleep()

FCUと接続するまで待ちます。
``current_state.connected`` はFCUと接続されているときに ``true`` になるので、接続されるとwhileループから抜けます。
また、 ``rate.sleep()`` は、このノードが20Hzで動作するように必要な時間だけノードを停止します。

.. code-block:: python

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    for i in range(100):
        local_pos_pub.publish(pose)
        r.sleep()

        if rospy.is_shutdown():
            break

あらかじめ移動先の位置をパブリッシュしておきます。
移動先の位置をパブリッシュしておかないとOffboardモードへの移行がリジェクトされます。
ここでは、移動先の位置は上方向に2mとなっています。
``PoseStamped`` の各フィールドについては `geometry_msgs/PoseStamped Message <http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html>`_ を見てください。

.. code-block:: python

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"

``SetMode`` メッセージの ``custom_mode`` フィールドを ``OFFBOARD`` にします。
このメッセージをサービスサーバに送信することでモードの切り替えをおこないます。
``SetMode`` メッセージの定義は `mavros_msgs/SetMode Service <http://docs.ros.org/api/mavros_msgs/html/srv/SetMode.html>`_ から見れます。

.. code-block:: python

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

``/mavros/cmd/arming`` サービスへ送信するためのメッセージを作成します。
``CommandBool`` メッセージの定義は `mavros_msgs/CommandBool Service <http://docs.ros.org/api/mavros_msgs/html/srv/CommandBool.html>`_ にあります。
mavrosパッケージで使用するメッセージの定義は、 `mavrosのROS wikiページ <http://wiki.ros.org/mavros#Plugins>`_ から見ることができます。

.. code-block:: python
    :linenos:

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" \
              and (rospy.Time.now() - last_request > rospy.Duration(5)):

            try:
                offb_set_mode_resp = set_mode_client(offb_set_mode)
                if offb_set_mode_resp.mode_sent:
                    rospy.loginfo("Offboard enabled")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()

        else:
          if not current_state.armed \
                and (rospy.Time.now() - last_request > rospy.Duration(5)):

            try:
                arm_cmd_resp = arming_client(arm_cmd)
                if arm_cmd_resp.success:
                    rospy.loginfo("Vehicle armed")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()

        local_pos_pub.publish(pose)
        r.sleep()ate.sleep();
    }

最初のif節は現在のモードが ``OFFBOARD`` かつ、モード切替が行われたのが5秒以上前であるときに実行されます。
最後のモード切替からの経過時間は ``last_request`` 変数を使って判断しています。

``set_mode_client(offb_set_mode)`` で、 ``offb_set_mode`` 変数のメッセージを使って ``mavros/set_mode`` サービスをコールしています。
サービスにはリクエストのフィールド（サービスコールの際に必要な情報を与えるために使う）と、レスポンスのフィールド（処理結果等を格納するのに使う）があり、サービスコールが成功するとレスポンスが帰ってきます。
今回はレスポンスの ``mode_sent`` を調べることでモード切り替えの命令がきちんと認識され、送信されたかを判断しています。

サービスコールが失敗した場合には例外が送出されるので、try-except節で例外をキャッチする必要があります。

モードが ``OFFBOARD`` の時は機体にモータを起動するように命令を送っています。
この部分の処理はOffboardコマンドを送るところとほぼ同じです。

最後に、ループ毎に目標位置をパブリッシュしています。


ビルドする
----------------------------------------------------------------
``offboard_sample.py`` が使用するメッセージが確実に生成されているようにするためにビルドします。

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make

実行権限の付与
----------------------------------------------------------------
PythonプログラムをROSノードとして実行するには実行権限を与える必要があります。

.. code-block:: bash

    chmod +x offboard_sample.py

ノードの起動
================================================================
はじめにシミュレータを起動します。
ROSノードとの通信を行うためにはmavrosが必要なので、 ``mavros_posix_sitl.launch`` を使います。

.. code-block:: bash

    roslaunch px4 mavros_posix_sitl.launch

次に先ほど作成したノードを起動します。

.. code-block:: bash

    rosrun px4_sim_pkg offboard_sample.py

.. image:: imgs/runningnode_py.png

launchファイルを書く
================================================================
シミュレータとOffboardノードを別々に起動するのは面倒なので、launchファイルにまとめて一つのコマンドで起動できるようにしましょう。

``px4_sim_pkg/launch`` 以下に ``py_offb_sample.launch`` という名前で以下の内容を保存してください。

.. code-block:: xml

    <launch>

        <include file="$(find px4)/launch/mavros_posix_sitl.launch" />
        <node name="offb_node" pkg="px4_sim_pkg" type="offboard_sample.py" />

    </launch>

以下のコマンドを使ってこのlaunchファイルからシミュレータとOffboardノードを起動できます。

.. code-block:: bash

    roslaunch px4_sim_pkg py_offb_sample.launch

参考
================================================================
`mavros - ROS Wiki <http://wiki.ros.org/mavros>`_
    mavrosのROS Wikiページ
`Pythonでのグローバル（global）変数の宣言方法 <https://uxmilk.jp/12505>`_
    グローバル変数について
`シンプルな配信者(Publisher)と購読者(Subscriber)を書く(Python) <http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_
    ROS Wikiのパブリッシャとサブスクライバを書くチュートリアル
`シンプルなサービスとクライアントを書く (Python) <http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28python%29>`_
    ROS Wikiのサービスとクライアントを書くチュートリアル
`rospy/Overview/Services <http://wiki.ros.org/rospy/Overview/Services>`_
    Overview of rospy Service
`Class ServiceProxy <http://docs.ros.org/api/rospy/html/rospy.impl.tcpros_service.ServiceProxy-class.html>`_
    Service Proxy API Reference