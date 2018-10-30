-------------------------------------
自律飛行とSLAM
-------------------------------------

このチュートリアルでは、`navigation stack <http://wiki.ros.org/navigation>`_ パッケージを使ってSLAMを行います。

はじめにGPSを用いた自律飛行、次に2D LiDARのみを用いた自己位置推定と自律飛行、最後に2D LiDARのみを用いたSLAM（Simultaneous Localization and Mapping）を行います。

はじめに
=====================================

.. figure:: imgs/overview_tf.png

    出典: http://wiki.ros.org/navigation/Tutorials/RobotSetup

Navigation stackは、ロボットの自律移動を行うために以下のようなサブパッケージを提供しています。

`move_base <http://wiki.ros.org/move_base>`_
    ロボットの経路計画に用いられる
`amcl <http://wiki.ros.org/amcl>`_
    ロボットの自己位置推定を行う


GPSを用いた自律飛行
=====================================

tfの設定
-------------------------------------

センサ情報の設定
-------------------------------------

オドメトリ情報の設定
-------------------------------------

移動制御の設定
-------------------------------------

LiDARとAMCLを用いた自己位置推定
=====================================

LiDARを用いたSLAM
=====================================
今回は `gmapping <http://wiki.ros.org/gmapping>`_ パッケージを使って地図生成を行います。


参考
=====================================
`Navigation stack tutorial <http://wiki.ros.org/navigation#Tutorials>`_
  navigation stackパッケージのチュートリアル
`move_base - ROS Wiki <http://wiki.ros.org/move_base>`_
  move_baseパッケージ
`2D Mapping & Navigation <https://www.wilselby.com/research/ros-integration/2d-mapping-navigation/>`_
  Navigation Stackを使ってSLAMをしている例
`How to Build a Map Using Logged Data - ROS Wiki <http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData>`_
  gmappingを使って自己位置推定用の地図を作る
`ROSのLidarSLAMまとめ - Qiita <https://qiita.com/nnn112358/items/814c0fb0d2075eb71da0>`_
  ROSで使えるLiDAR SLAM一覧
`Navigation Stack を理解する - 1. 導入 - Qiita <https://qiita.com/MoriKen/items/0b75ab291ab0d95c37c2P>`_
  Navigation Stackを使ってみる