PX4 SITL Tutorial
*****************************************************************
このテキストについて
=================================================================
このテキストはPX4 SITLシミュレーションを用いてドローンアプリケーション開発を行いたい人向けのチュートリアルです。
本テキスト内では、

* シミュレーション環境の構築
* シミュレーションの実行
* 自作ノードの実行
* ドローンの自律移動
* Octomapを用いた3D地図生成
* MoveIt!を用いた動作計画

について説明をします。

SLAMセクションを最初から読んでいることが前提で書いているので、
他のセクションから読み始めると内容がわからないことがあるかもしれません。

また、読書案内のページで補足資料等をリストアップしてあるので、必要に応じて参考にしてください。

対象読者
=================================================================
このテキストの対象読者は、

1. PX4ファームウェアを用いてドローンアプリケーション開発を行いたい人
2. ドローンのシミュレーションを行いたい人

です。

環境
=================================================================
本テキストにおいては以下の環境を使用しています。

* Ubuntu 16.04 LTS
* ROS Kinetic
* PX4 Firmware v1.8.0

ソースコード
=================================================================
このチュートリアルで使用したプログラムやLaunchファイルなどは `dronedocリポジトリ <https://github.com/uenota/dronedoc>`_ にあります。

.. toctree::
   :maxdepth: 2
   :caption: SLAM

   px4sim/px4sim.rst
   runnode/runnodecpp.rst
   runnode/runnodepy.rst
   addmodel/addmodel.rst
   slam/slam.rst
   slam/gps_nav.rst
   slam/turtle_mapping.rst
   teleop/teleop.rst
   slam/lidar_localization.rst
   slam/localization_and_mapping.rst
   slam/lidar_nav.rst

.. toctree::
   :maxdepth: 2
   :caption: Mapping

   octomap/octomap.rst
   build_map_gmapping/build_map_gmapping.rst
   build_map_gazebo_plugin/build_map_gazebo_plugin.rst

.. toctree::
   :maxdepth: 2
   :caption: Motion Planning

   moveit/index.rst
   local_planner_for_uav/local_planner_for_uav.rst

.. toctree::
   :maxdepth: 2
   :caption: Tips

   qgc_intro/qgc_intro.rst
   add_range_sensor/add_range_sensor.rst
   filter_lidar/filter_lidar.rst
   change_local_planner/change_local_planner.rst
   airsim/airsim.rst

.. toctree::
   :maxdepth: 1
   :caption: Appendix

   appendix/bash_commands/bash_commands.rst
   appendix/ros/ros.rst
   appendix/terms/terms.rst
   appendix/archive/archive.rst
   appendix/trouble_shooting/index.rst

.. **Index**

.. * :ref:`genindex`

Issues & Pull Requests
====================================================
内容に問題があれば気軽にお知らせください。

https://github.com/uenota/dronedoc/issues


Pull Requestも大歓迎です!!!

https://github.com/uenota/dronedoc/pulls


License
====================================================

.. image:: https://i.creativecommons.org/l/by/4.0/88x31.png
  :target: http://creativecommons.org/licenses/by/4.0/
  :alt: Creative Commons License

This work is licensed under a `Creative Commons Attribution 4.0 International License <http://creativecommons.org/licenses/by/4.0/>`_.