==========================================================
地上からの距離を一定にする
==========================================================

距離センサをつける
==========================================================
モデルを作成する
----------------------------------------------------------
レーザー高度計（Lidar Liteなど）とRayセンサ（Lidarなど）を同時に使用するとセグメンテーションフォールトでGazeboが起動できない（ `#9156 <https://github.com/PX4/Firmware/issues/9156>`_）ので今回は距離センサとして超音波センサを利用します。
GpuRayセンサであれば問題なく起動できるらしいので、どうしてもレーザー高度計を使用したい場合にはGpuRayセンサを利用すると良いでしょう。
ただし、GpuRayセンサはGPUを使用しない場合は正常に動作しないようなので、GPUがある場合に限って使用しましょう。

また、iris_2d_lidarモデルをインクルードして超音波センサをつけても良いのですが、名前空間が煩雑（iris_2d_lidar::iris::base_linkとか）になり、TFの設定などが面倒なので、iris_2d_lidarモデルのSDFファイルをコピーして超音波センサを追加します。

超音波センサを追加したモデルは以下のようになります。

.. literalinclude:: ../../../../models/iris_sonar/model.sdf
    :language: xml
    :linenos:
    :caption: model.sdf

以下のようなmodel.configも作成し、modelsディレクトリ以下にiris_sonarディレクトリを作ってその中に保存しましょう。
autrhorタグの中身は適宜変更してください。

.. literalinclude:: ../../../../models/iris_sonar/model.config
    :language: xml
    :linenos:
    :caption: model.config

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

地上からの距離を一定にする
----------------------------------------------------------
.. TODO:パラメータ変えればいけるはず

..
    TODO:
    ノードを書く場合はメッセージをパブリッシュするように設定する
    mavlinkの設定(DISTANCE_SENSOR...)を.postファイルで行う必要？
