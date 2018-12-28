:tocdepth: 1

UAV用のローカルプランナを実装する
==========================================================

この章では、`Open Motion Planning Library (OMPL) <http://ompl.kavrakilab.org/>`_ を使ってドローン用のローカルプランナを実装します。

OMPLはオープンソースの行動計画ライブラリで、マニピュレータやUGVなどの動作計画に用いられる `MoveIt! <https://moveit.ros.org/>`_ というライブラリもOMPLを使っています。

インストール
----------------------------------------------------------
UbuntuにOMPLをインストールする方法は以下の3通りあります。

* ソースからインストールする
* ビルド済みバイナリをインストールする
* ROSバージョンのバイナリをインストールする

今回はROSバージョンのバイナリをインストールします。
他の方法については、`OMPLのインストールガイド <http://ompl.kavrakilab.org/installation.html>`_ を見てください。

はじめに、ROSのリポジトリをaptのデータ取得元に追加します。
すでにROSを使っている場合には必要ありません。

.. code-block:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

追加できたら、OMPLをインストールします。

.. code-block:: bash

    sudo apt update
    sudo apt install ros-kinetic-ompl

ローカルプランナを実装する
----------------------------------------------------------
以下では、`制御指令値を含むプラニング <https://robotics.naist.jp/edu/text/?Robotics%2FOMPL#PlanningWithControls>`_ を参考にして作業を進めていきます。

CMakeLists.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: cmake

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        ...
        nav_core    # add
        tf          # add
        costmap_2d  # add
    )

.. code-block:: cmake

    find_package(OMPL REQUIRED)

.. code-block:: cmake

    add_executable(rrt_planner src/rrt_planner.cpp)
    target_link_libraries(rrt_planner ${catkin_LIBRARIES}
                                      ${OMPL_LIBRARIES}
                                      ${Boost_LIBRARIES})

ROSプラグイン
----------------------------------------------------------
ROSのパッケージの中には、処理の一部（画像処理や行動計画など）をプラグインとして作成し、後で変更可能になっているものがあります。
プラグインを作成、使用するためには `pluginlib <http://wiki.ros.org/pluginlib>`_ パッケージを使用します。
プラグインの仕組みを使うことで、特定のインターフェースに従ったクラスを作れば中身の実装は自由にできるので、既存のパッケージを拡張することが容易になります。

navigationスタック
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
navigationスタックは、move_baseやcostmap_2dなどを含むメタパッケージです。

navigationスタックでは、pluginlibを使って作成したプラグインをローカルプランナやグローバルプランナとして使用することができるようになっています。
navigationスタックのローカルプランナとして使用可能なクラスを作るためには、 `nav_core::BaseLocalPlanner <http://docs.ros.org/jade/api/nav_core/html/classnav__core_1_1BaseLocalPlanner.html>`_ を継承したクラスを作る必要があります。

ローカルプランナをプラグイン化する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


実行する
----------------------------------------------------------

参考
----------------------------------------------------------
`Robotics/OMPL - NAIST::OnlineText <https://robotics.naist.jp/edu/text/?Robotics%2FOMPL>`_
    OMPLの日本語チュートリアル
`Open Motion Planning Library: A Primer <http://ompl.kavrakilab.org/OMPL_Primer.pdf>`_
    OMPLの英語チュートリアル（PDF）
`sources.list - APT のデータ取得元の設定リスト <http://manpages.ubuntu.com/manpages/xenial/ja/man5/sources.list.5.html>`_
    sources.listについて
`navigationスタックで学ぶpluginlibの使い方 <http://makemove.hatenablog.com/entry/2015/09/29/001725>`_
    navigationスタック用のプラグインについて
`Writing A Global Path Planner As Plugin in ROS <http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS>`_
    navigationスタック用のグローバルプランナプラグインを作る
`nav_core - ROS Wiki <http://wiki.ros.org/nav_core>`_
    nav_coreパッケージ
`navigation/dwa_local_planner - GitHub <https://github.com/ros-planning/navigation/tree/melodic-devel/dwa_local_planner>`_
    dwa_local_plannerのソースコード
