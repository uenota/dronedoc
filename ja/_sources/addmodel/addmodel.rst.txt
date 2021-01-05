:tocdepth: 1

新しいモデルを追加する
*****************************************************************

この章では、次の章で使用する、2D Lidar付きのIrisのモデルを作成します。

はじめに、Irisのモデルを格納するディレクトリを作成します。
今回は、iris_2d_lidarというフォルダに格納することにします。

.. code-block:: bash

    cd ~/catkin_ws/src/px4_sim_pkg
    mkdir -p models/iris_2d_lidar

SDFファイルを作る
=======================================================================

SDFとは
-----------------------------------------------------------------------
Gazeboでは、SDFというフォーマットを使用してロボットのモデルを定義します。
SDFとURDFの関係については `Gazebo + ROS で自分だけのロボットをつくる 4.URDFファイルをつくる <https://qiita.com/RyodoTanaka/items/174e82f06b10f9885265>`_ を参照してください。
また、GazeboでURDFを使う場合には `Tutorial: Using a URDF in Gazebo <http://gazebosim.org/tutorials/?tut=ros_urdf>`_ が参考になります。

model.sdf
-----------------------------------------------------------------------
先ほど作ったiris_2d_lidarフォルダ以下にmodel.sdfファイルを作成します。
これがロボットの物理モデルや形状を定義するファイルです。

既存のモデルをインクルードする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
SDFでは既存のモデルをインクルードして再利用することができます。
以下のように記述することで既存のモデルをインクルードできます。

.. code-block:: xml
    :linenos:

    <include>
      <uri>model://iris</uri>
    </include>

今回はPX4 Firmwareにあるirisのモデルを使用します。
このようにインクルードする場合には、 ``GAZEBO_MODEL_PATH`` に使用するモデルのパスが設定されていないといけません。
このチュートリアル通りに環境構築をしている場合にはすでにモデルのパスが設定されているはずですが、必要な場合は以下のコマンドを実行してください。

.. code-block:: bash

    export GAZEBO_MODEL_PATH=$HOME/src/Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH

LiDARのリンクを定義する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
リンク（ ``<link></link>`` ）はモデルの物理特性を定義するために使います。
できるだけ少ない数のリンクでモデルを構築したほうがシミュレーションのパフォーマンスや安定性が向上します。

model.sdfファイル内では、以下のようにLiDARのリンクを定義しています。

.. literalinclude:: ../../models/iris_2d_lidar/model.sdf
    :linenos:
    :language: xml
    :lines: 9-70

それぞれの要素について見ていきましょう。

inertial
    質量や慣性モーメントなどの動力学的特性を指定します。
visual
    形状やテクスチャなどの外見を指定します。 ``geometry`` で形状、 ``material`` でテクスチャを指定します。形状には円柱や直方体の他に、3D CADなどで作成したメッシュを指定できます。
sensor
    センサの仕様や使うプラグインを指定します。使えるセンサにはLiDARやカメラ、IMU等があります。
collision
    接触判定に使われるリンクの形状を指定します。 ``visual`` タグと同様にシンプルな形状の他にメッシュを指定することもできます。複雑なコリジョンの指定はパフォーマンスに影響を与えるので、円柱や直方体など、できるだけシンプルな形にしましょう。シミュレーション内のセンサの値が狂う場合もあります。今回のモデルでは使われていません。
plugin
    センサの動作をシミュレートするためのものです。今回はレーザのプラグインを使っています。詳細は `Tutorial: Using Gazebo plugins with ROS <http://gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser>`_ を見てください。

LiDARとbase_linkのジョイントを定義する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ジョイントは2つのリンクを接続するための要素です。
今回はirisのbase_linkリンクとLiDARのlidar_linkリンクを接続します。

``<child>`` タグで子リンクを指定し、 ``<parent>`` タグで親リンクを指定しています。
irisのbase_linkのような、インクルードしたモデルのリンクを指定する際には、 ``iris::base_link`` のように、どのモデルのリンクであるかを指定してやる必要があります。

.. literalinclude:: ../../models/iris_2d_lidar/model.sdf
    :language: xml
    :linenos:
    :lines: 72-75

今回は固定関節を使っていますが、他にも回転関節や直動関節などが使えます。

model.sdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
最終的なmodel.sdfファイルは以下のようになります。

.. literalinclude:: ../../models/iris_2d_lidar/model.sdf
    :language: xml
    :linenos:
    :caption: model.sdf

lidarのリンクの姿勢は、

.. code-block:: xml

    <pose>0 0 0.05 0 -0.0085 0</pose>

のように、少し後ろに傾けて定義してあります。
これは、ドローン自身に光線が干渉するのを防ぐためです。

SDFのタグ一覧は `ここ <http://sdformat.org/spec>`_ から見ることができます。

LiDARのモデルも別のファイルで記述してインクルードするという形にすると、モデルの記述がすっきりしていいかもしれません。
その場合は、 ``<link name="lidar_link>..</link>`` で囲まれている部分を別のモデルに分離して、インクルードするという形になります。
ジョイントはリンク同士の接続を定義しているので、そのままにします。

例えばこんな感じです。

.. code-block:: xml
    :linenos:

    <?xml version='1.0'?>
    <sdf version='1.6'>
    <model name='iris_2d_lidar'>

        <include>
        <uri>model://iris</uri>
        </include>

        <include>
        <uri>model://lidar_2d</uri>
        <pose>0 0 0.07 0 0 0</pose>
        </include>

        <joint name="lidar_joint" type="fixed">
        <child>lidar_2d::lidar_link</child>
        <parent>iris::base_link</parent>
        <axis>
            <xyz>0 0 1</xyz>
            <limit>
            <upper>0</upper>
            <lower>0</lower>
            </limit>
        </axis>
        </joint>

    </model>
    </sdf>

この例ではLiDARはlidar_2dというモデルに分離しています。
別のファイルにモデルを分割する場合は、 ``GAZEBO_MODEL_PATH`` にモデルがあるパスを追加することを忘れないでください。
例えば、 ``px4_sim_pkg/models`` 以下にlidar_2dモデルを置いた場合には、以下のコマンドを実行して設定を行います。

.. code-block:: bash

    export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/px4_sim_pkg/models:$GAZEBO_MODEL_PATH

ターミナルからコマンドを実行して設定を行った場合は毎回設定する必要がありますが、.bashrcにこの内容を記述しておけば、ターミナルを起動した時に自動で設定されます。


configファイルを作る
======================================================================
モデルには、名前や作成者などのモデルの情報を記したmodel.configファイルが必要です。
基本的には以下のような形になります。
タグの詳細については `Model structure and requirements <http://gazebosim.org/tutorials?tut=model_structure>`_ で確認してください。

.. literalinclude:: ../../models/iris_2d_lidar/model.config
    :language: xml
    :linenos:
    :caption: model.config

パスを設定する
======================================================================
GazeboのGUIからモデルを追加したり、SDFファイルからインクルードするためには、作成したモデルのパスが通っていないといけません。
モデルのパスは ``~/catkin_ws/src/px4_sim_pkg/models`` なので、次のコマンドを実行してモデルのパスを設定します。

.. code-block:: bash

    export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/px4_sim_pkg/models:$GAZEBO_MODEL_PATH

もしくは、上のコマンドをそのまま ``~/.bashrc`` に書いても構いません。

モデルを確認する
======================================================================
GazeboのGUIからモデルを読み込んで確認してみましょう。

.. code-block:: bash

    gazebo


.. image:: imgs/model1.png

赤枠内のInsertタブからiris_2d_lidarを選択してモデルを追加しましょう。

.. image:: imgs/model2.png

LiDAR付きのIrisのモデルがきちんと作成されたことが確認できました。

SITLシミュレーションでモデルを使う
======================================================================
次のコマンドを実行すると今回作成したモデルをSITLシミュレーションで使うことができます。
コマンドラインからlaunchファイルに引数を与える方法については、 `ROS Wiki <http://wiki.ros.org/roslaunch/XML/arg#Passing_an_argument_via_the_command-line>`_ を見てください。

.. code-block:: bash

    roslaunch px4 posix_sitl.launch sdf:=$HOME/catkin_ws/src/px4_sim_pkg/models/iris_2d_lidar/model.sdf

mavros_posix_sitl.launchを使う場合には以下のようにします。

.. code-block:: bash

    roslaunch px4 mavros_posix_sitl.launch sdf:=$HOME/catkin_ws/src/px4_sim_pkg/models/iris_2d_lidar/model.sdf

.. image:: imgs/sitl.png

もしくは、このようなlaunchファイルを書いてもいいでしょう。

.. code-block:: xml
    :linenos:

    <launch>

        <arg name="sdf" default="$(find dronedoc)/models/iris_2d_lidar/model.sdf" />

        <include file="$(find px4)/launch/mavros_posix_sitl.launch" >
            <arg name="sdf" value="$(arg sdf)" />
        </include>

    </launch>

参考
====================================================================
`Make a Model <http://gazebosim.org/tutorials?tut=build_model>`_
    Gazeboモデルの作成について
`Make a Mobile Robot <gazebosim.org/tutorials?tut=build_robot>`_
    モバイルロボットの作成について
`Model structure and requirements <http://gazebosim.org/tutorials?tut=model_structure>`_
    モデルのファイル構成について
`Gazebo Components <http://gazebosim.org/tutorials?tut=components#EnvironmentVariables>`_
    GAZEBO_MODEL_PATHについて