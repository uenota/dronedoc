MoveIt!のRvizデモを試す
=================================================================

このページでは、:doc:`../create_config_pkg/create_config_pkg` で作成したiris_moveit_configパッケージ内にあるRvizプラグインのデモを実行します。

demo.launch
-----------------------------------------------------------------
以下のコマンドでデモノードを起動できます。

.. code-block:: bash

    roslaunch iris_moveit_config demo.launch

デモが起動したら以下のようなウィンドウが表示されます。

.. image:: imgs/demo.png

マーカーを使ってドローンのゴールの位置と姿勢を指示します。

.. image:: imgs/goal_state.png

次に、スタートの位置と姿勢を指示するために、"Motion Planning"->"Planning Request"以下の"Query Start State"にチェックを入れます。

.. image:: imgs/planning_request.png

新しくマーカーが表示されるので、先ほどと同様にしてスタートの位置を選択します。

.. image:: imgs/set_start.png

スタートとゴールの位置を選択したら、"Motion Planning"サブウィンドウの"Planning Library"からプランナを設定します。
また、"Workspace"から、プランナが経路計画を領域のサイズを変更します。

.. image:: imgs/planner_config.png

設定が完了したら、"Planning"タブ内の"Plan"ボタンをクリックすると経路計画がスタートします。

.. image:: imgs/planning.png

経路計画が完了したら以下のように計算された経路が表示されます。

.. image:: imgs/trajectory.png

以下は一連の手順をまとめた動画です。

.. image:: imgs/demo.gif

参考
-----------------------------------------------------------------
`MoveIt! Quickstart in RViz <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_