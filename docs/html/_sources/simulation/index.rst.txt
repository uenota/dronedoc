##########################################
Simulation
##########################################

**このテキストについて**

このテキストはPX4 SITLシミュレーションを用いてドローンアプリケーション開発を行いたい人向けのチュートリアルです。
本テキスト内では、

* シミュレーション環境の構築
* シミュレーションの実行
* 自作ノードの実行
* ドローンの自律移動
* Octomapを用いた3D地図生成

について説明をします。

また、読書案内のページで補足資料等をリストアップしてあるので、必要に応じて参考にしてください。


**対象読者**

このテキストの対象読者は、

1. PX4ファームウェアを用いてドローンアプリケーション開発を行いたい人
2. ドローンのシミュレーションを行いたい人

です。

**環境**

本テキストにおいては以下の環境を使用しています。

* Ubuntu 16.04 LTS
* ROS Kinetic
* PX4 Firmware v1.8.0

**ソースコード**

このチュートリアルで使用したプログラムやLaunchファイルなどは `dronedocリポジトリ <https://github.com/uenota/dronedoc>`_ にあります。

.. toctree::
   :maxdepth: 1
   :caption: Contents
   :numbered:

   px4sim/px4sim.rst
   runnode/runnodecpp.rst
   runnode/runnodepy.rst
   addmodel/addmodel.rst
   slam/slam.rst
   slam/gps_nav.rst
   slam/lidar_localization.rst
   slam/localization_and_mapping.rst
   octomap/octomap.rst

**Tips**

* :doc:`tips/control_alt/control_alt`
* :doc:`tips/filter_lidar/filter_lidar`
* :doc:`tips/change_local_planner/change_local_planner`
* :doc:`tips/local_planner_for_uav/local_planner_for_uav`
* :doc:`tips/build_map_gmapping/build_map_gmapping`
* :doc:`tips/build_map_gazebo_plugin/build_map_gazebo_plugin`