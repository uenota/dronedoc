:tocdepth: 1

自律飛行とSLAM
*************************************

このチュートリアルでは、`navigation stack <http://wiki.ros.org/navigation>`_ パッケージを使ってSLAMを行います。

はじめにGPSを用いた自律飛行、次に2D LiDARのみを用いた自己位置推定と自律飛行、最後に2D LiDARのみを用いたSLAM（Simultaneous Localization and Mapping）を行います。

はじめに
=====================================

.. figure:: imgs/overview_tf.png

    From `ROS Wiki <http://wiki.ros.org/navigation/Tutorials/RobotSetup>`_ (`CC BY 3.0 <https://creativecommons.org/licenses/by/3.0/>`_)

Navigation stackは、ロボットの自律移動を行うために以下のようなサブパッケージを提供しています。

`move_base <http://wiki.ros.org/move_base>`_
    ロボットの経路計画に用いられる
`amcl <http://wiki.ros.org/amcl>`_
    ロボットの自己位置推定を行う


navigation stackのインストール
-------------------------------------

.. code-block:: bash

    sudo apt install ros-kinetic-amcl \
                     ros-kinetic-base-local-planner \
                     ros-kinetic-carrot-planner \
                     ros-kinetic-clear-costmap-recovery \
                     ros-kinetic-costmap-2d \
                     ros-kinetic-dwa-local-planner \
                     ros-kinetic-fake-localization \
                     ros-kinetic-global-planner \
                     ros-kinetic-map-server \
                     ros-kinetic-move-base \
                     ros-kinetic-move-base-msgs \
                     ros-kinetic-move-slow-and-clear \
                     ros-kinetic-nav-core \
                     ros-kinetic-navfn \
                     ros-kinetic-rotate-recovery \
                     ros-kinetic-voxel-grid

参考
=====================================
`Navigation stack tutorial <http://wiki.ros.org/navigation#Tutorials>`_
  navigation stackパッケージのチュートリアル

`2D Mapping & Navigation <https://www.wilselby.com/research/ros-integration/2d-mapping-navigation/>`_
  Navigation Stackを使ってSLAMをしている例

`Navigation Stack を理解する - 1. 導入 - Qiita <https://qiita.com/MoriKen/items/0b75ab291ab0d95c37c2P>`_
  Navigation Stackを使ってみる