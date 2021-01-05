:tocdepth: 1

ゲームパッドを使ってドローンを操作する
**********************************************************************

このページでは、ゲームパッド（ジョイスティック）と `mavros_extras <http://wiki.ros.org/mavros_extras>`_ パッケージのmavteleopノードを使ってドローンを操作する方法について解説します。

.. _start_joy:

joy_nodeを起動する
======================================================================
joy_nodeは `joy <http://wiki.ros.org/joy>`_ パッケージに含まれるノードで、ゲームパッドの入力を読み取って ``sensor_msgs/Joy`` 型のメッセージをパブリッシュしてくれます。

使用するには、ゲームパッドを接続して以下のコマンドを実行します。

.. code-block:: bash

    roscore
    rosrun joy joy_node

ノードが正常に実行されている場合は、ゲームパッドを操作すると ``/joy`` トピックに以下のようなメッセージがパブリッシュされます。

.. code-block:: none

    header:
    seq: 69
    stamp:
        secs: 1544540912
        nsecs: 248934345
    frame_id: ''
    axes: [-0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
    buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ---
    header:
    seq: 70
    stamp:
        secs: 1544540912
        nsecs: 312954596
    frame_id: ''
    axes: [-0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
    buttons: [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ---

mavteleopノードを起動する
======================================================================
mavteleopノードはゲームパッドの入力を、ドローンの制御入力に変換して、 ``setpoint_*`` トピック（ ``setpoint_velocity`` 、 ``setpoint_attitude`` 、 ``setpoint_position`` など）にパブリッシュすることでドローンを操作します。

与える引数によって操作方法を変えることができるので、今回は ``-vel`` オプションを与えて、ドローンの速度を操作するようにします。

.. code-block:: bash

    rosrun mavros_extras mavteleop -vel

``-vel`` オプション以外にも ``-att`` や ``-pos`` オプションがあり、それぞれ姿勢と位置を操作するモードです。

パラメータの設定用ファイルを作成する
======================================================================
ゲームパッドのマッピングを確認する
----------------------------------------------------------------------
:ref:`start_joy` で説明したように、joy_nodeノードを起動すると、 ``/joy`` トピックにジョイスティックの入力値がパブリッシュされます。
パブリッシュされているメッセージの、axesがスティックの入力、buttonsがボタンの入力です。 それぞれインデックスは0から始まるので注意してください。
``/joy`` トピックの内容を確認しながらゲームパッドを操作することで、ゲームパッドのマッピングを確認することができます。

このような、ゲームパッドのハードウェア（ボタンやスティックなど）とOSが認識するボタンのIDの対応関係のことをマッピングと言います。

YAMLファイルを作る
----------------------------------------------------------------------
ROSでは、YAMLファイルを使ってパラメータをまとめてロードしたり、保存したりできます（ `参考 <http://wiki.ros.org/rosparam>`_ ）。
今回はこの仕組みを使ってゲームパッドのマッピングに関するパラメータをまとめてロードさせます。

以下は、 `Logicool F310 <https://gaming.logicool.co.jp/ja-jp/products/gamepads/f310-gamepad.html>`_ 用のパラメータが記述されたファイルです。
このファイル内に書かれた内容がROSのパラメータとして保存され、mavteleopノードによって参照されます。
自分の使っているゲームパッドのマッピングと合うように、 ``axes_map`` と、 ``button_map`` の部分を変更しましょう。
また、 ``axes_scale`` を変更すればスティックの操作量とドローンの移動速度の対応を変化させられます。

.. literalinclude:: ../../config/f310_joy.yaml
  :linenos:
  :language: yaml
  :caption: f310_joy.yaml

パラメータをロードする
----------------------------------------------------------------------

パラメータをロードするには以下のコマンドを使います。

.. code-block:: bash

    rosparam load ~/catkin_ws/src/px4_sim_pkg/config/f310_joy.yaml mavteleop

``rosparam load ファイル ネームスペース`` のように記述します。
ネームスペースは省略できます。

実行する
======================================================================
はじめにPX4 SITLシミュレーションを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg mymodel_sitl_tf.launch

次に、joy_nodeノードを起動します。
Offboardコマンドは500 msの間入力がなければタイムアウトするので、 ``cmd_vel`` トピックに5 Hzの周期でメッセージをパブリッシュするために、 ``autorepeat_rate`` パラメータを5に設定しています。

.. code-block:: bash

    rosrun joy joy_node _autorepeat_rate:=5

mavteleopノード用のパラメータをロードします。

.. code-block:: bash

    rosparam load ~/catkin_ws/src/px4_sim_pkg/config/f310_joy.yaml mavteleop

mavteleopノードを起動します。

.. code-block:: bash

    rosrun mavros_extras mavteleop -vel

ゲームパッドからドローンを操作できるようにするためには、OFFBOARDモードである必要があるので、離陸させてからモードを変更します。

.. code-block:: bash

  rosrun mavros mavsys mode -c OFFBOARD

OFFBOARDモードに変更するためには、すでに ``setpoint_*`` トピックにメッセージが届いている必要があります。
この場合は、mavteleopノードが ``/mavros/setpoint_velocity/cmd_vel`` トピックにメッセージをパブリッシュしているので、モード変更することができます。
自作のノードなどでOFFBOARDモードに変更できない時は、 ``setpoint_*`` トピックにメッセージがパブリッシュされているか確認しましょう。

ゲームパッドを操作すれば以下のようにドローンが移動するはずです。

.. image:: imgs/teleop.gif


Launchファイルを使う
======================================================================
Launchファイルを使えばjoy_nodeノードとmavteleopノードの起動と同時にパラメータを設定することができます。
mavteleopノードを使用するLaunchファイルを以下に示します。

.. code-block:: xml
  :linenos:
  :caption: mavros_teleop.launch

  <launch>
    <arg name="teleop_args" default="-vel" />

    <node pkg="joy" type="joy_node" name="joy" required="True">
      <param name="autorepeat_rate" value="5" /> <!-- Minimal update rate, Hz -->
    </node>

    <node pkg="mavros_extras" type="mavteleop" name="mavteleop" args="$(arg teleop_args)" required="True" output="screen">
      <rosparam command="load" file="$(find px4_sim_pkg)/config/f310_joy.yaml" />
    </node>
  </launch>

mavros_teleop.launch内ではゲームパッドのマッピングを記述したファイル（f310_joy.yaml）を ``rosparam`` タグでROSのパラメータとしてロードしています。

実行する
----------------------------------------------------------------------
Launchファイルを使う場合には、以下の手順だけで実行できます。

はじめにPX4 SITLシミュレーションを起動します。

.. code-block:: bash

    roslaunch px4_sim_pkg mymodel_sitl_tf.launch

次に、joy_nodeノードとmavteleopノードを起動するLaunchファイルを実行します。
このLaunchファイル内で必要なパラメータの設定が行われるので、今回はコマンドラインからパラメータを設定する必要はありません。

.. code-block:: bash

    roslaunch px4_sim_pkg mavros_teleop.launch

最後に、ドローンが離陸してからモードを変更します。

.. code-block:: bash

  rosrun mavros mavsys mode -c OFFBOARD

参考
======================================================================
`rosparamの使い方メモ <https://qiita.com/honeytrap15/items/550c757f2964b575883c>`_
    rosparamについて