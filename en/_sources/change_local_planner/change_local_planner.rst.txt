:tocdepth: 1

move_baseのローカルプランナーを変更する
=================================================================
このページではmove_baseのローカルプランナーを変更する方法について扱います。

move_baseのローカルプランナーは、デフォルトでは `Trajectory Rollout <http://wiki.ros.org/base_local_planner>`_ が使用されています。
それ以外には、 `Dynamic Window Approach <http://wiki.ros.org/dwa_local_planner>`_ や、 `Elastic Band <http://wiki.ros.org/eband_local_planner>`_ 、 `Timed Elastic Band <http://wiki.ros.org/teb_local_planner>`_ などを使用することができます。

Dynamic Window Approach
-----------------------------------------------------------------
Dynamic Window Approach (DWA)は、base_local_plannerパッケージの `dwa` パラメータを `true` にすることで使用できます。

Elastic Band
-----------------------------------------------------------------
eband_local_plannerをインストールする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    sudo apt install ros-kinetic-eband-local-planner

設定ファイルを作る
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
以下の設定ファイルの内容を、config以下にeband_local_planner_params.yamlとして保存します。

eband_local_plannerのパラメータ一覧は `eband_local_plannerのROS Wikiページ <wiki.ros.org/eband_local_planner>`_ にあります。

.. literalinclude:: ../../config/eband_local_planner_params.yaml
    :linenos:
    :language: yaml
    :caption: eband_local_planner_params.yaml

move_baseのパラメータを変更する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
move_baseがEBandLocalPlannerROSを使うように設定します。
navigation.launchをコピーして新たにeband_navigation.launchを作成します。

`<node>` タグ内を以下のように変更します。

.. code-block:: xml
  :caption: eband_navigation.launch

  <node pkg="move_base" type="move_base" name="move_base">
      <param name="base_local_planner" value="eband_local_planner/EBandPlannerROS" />
      ...
      <rosparam command="load" file="$(find px4_sim_pkg)/config/eband_local_planner_params.yaml"/>
      ...
  </node>


Timed Elastic Band
-----------------------------------------------------------------
teb_local_plannerをインストールする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash

    sudo apt install ros-kinetic-teb-local-planner

設定ファイルを作る
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
以下の設定ファイルの内容を、config以下にteb_local_planner_params.yamlとして保存します。

teb_local_plannerのパラメータ一覧は `teb_local_plannerのROS Wikiページ <wiki.ros.org/teb_local_planner>`_ にあります。

.. literalinclude:: ../../config/teb_local_planner_params.yaml
    :linenos:
    :language: yaml
    :caption: teb_local_planner_params.yaml

move_baseのパラメータを変更する
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
move_baseがTebLocalPlannerROSを使うように設定します。
navigation.launchをコピーして新たにteb_navigation.launchを作成します。

`<node>` タグ内を以下のように変更します。

.. code-block:: xml
  :caption: teb_navigation.launch

  <node pkg="move_base" type="move_base" name="move_base">
      <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS" />
      ...
      <rosparam command="load" file="$(find px4_sim_pkg)/config/teb_local_planner_params.yaml"/>
      ...
  </node>

参考
-----------------------------------------------------------------
`Trajectory Rollout <https://www.semanticscholar.org/paper/Planning-and-Control-in-Unstructured-Terrain-Gerkey/dabdbb636f02d3cff3d546bd1bdae96a058ba4bc>`_
    Trajectory Rolloutアルゴリズム
`Dynamic Window Approach <https://doi.org/10.1109/100.580977>`_
    DWAアルゴリズム
`Elastic Band <https://doi.org/10.1109/ROBOT.1993.291936>`_
    Elastic Bandアルゴリズム
`Timed Elastic Band <https://doi.org/10.1016/j.robot.2016.11.007>`_
    Timed Elastic Bandアルゴリズム。他の論文は `GitHubページ <https://github.com/rst-tu-dortmund/teb_local_planner>`_ を参照
`Configure and run Robot Navigation <http://wiki.ros.org/teb_local_planner/Tutorials/Configure%20and%20run%20Robot%20Navigation>`_
    Timed Elastic Bandをmove_baseで使う
`dwa_planner vs. base_local_planner <https://answers.ros.org/question/10718/dwa_planner-vs-base_local_planner/>`_
    Dynamic Window ApproachとTrajectory Rolloutの比較
`difference between eband_local_planner and teb_local planner <https://answers.ros.org/question/242067/difference-between-eband_local_planner-and-teb_local-planner/>`_
    Elastic BandとTimed Elastic Bandの比較