地図を生成するGazebo Pluginを作る
=================================================================

以下のコード以外は `Custom messages <http://gazebosim.org/tutorials?cat=install&tut=custom_messages&ver=1.9-7>`_ にあるものと同じです。

map_builder.cpp
----------------------------------------------------------

.. literalinclude:: ../../src/gazebo/map_builder.cpp
    :linenos:
    :language: cpp
    :caption: map_builder.cpp

request_publisher.cpp
----------------------------------------------------------

.. literalinclude:: ../../src/gazebo/request_publisher.cpp
    :linenos:
    :language: cpp
    :caption: request_publisher.cpp

使用法
----------------------------------------------------------
プラグインを埋め込んだワールドを起動した状態で、次のようにノードを実行することでマップを作成できます。
以下のコマンドは一例です。引数無しで実行すれば使用法が表示されるので参考にしてください。

.. code-block:: bash

    rosrun dronedoc request_publisher 10 0.01 "(-12,0)" 60 50 ~/map.png 255

参考
----------------------------------------------------------
`Custom messages <http://gazebosim.org/tutorials?cat=install&tut=custom_messages&ver=1.9-7>`_
  カスタムメッセージを使ってコリジョンマップを作る
`ROS Plugin <http://gazebosim.org/tutorials?tut=ros_plugins>`_
  ROS用のGazeboプラグインを作る