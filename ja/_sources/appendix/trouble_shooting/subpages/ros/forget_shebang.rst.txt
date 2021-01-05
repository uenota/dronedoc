Pythonノードを実行したらimport: not authorized `rospy' @ error/constitute.c/WriteImage/1028.のエラーが出る
===============================================================================================================================
症状
-----------------------------------------------------------------
Pythonで書かれたROSノードを実行すると以下のようなエラーが出る。

.. code-block:: none

  import: not authorized `rospy' @ error/constitute.c/WriteImage/1028.
  from: can't read /var/mail/nav_msgs.msg
  from: can't read /var/mail/geometry_msgs.msg
  import: not authorized `tf' @ error/constitute.c/WriteImage/1028.
  /home/uenot/catkin_ws/src/dronedoc/script/odom_publisher.py: line 8: syntax error near unexpected token `('
  /home/uenot/catkin_ws/src/dronedoc/script/odom_publisher.py: line 8: `local_pos = PoseStamped()'

もしかして？
-----------------------------------------------------------------
Pythonプログラムの先頭に ``#!/usr/bin/env python`` と書くのを忘れていませんか？

`書いてあるのにエラーが出る場合 <https://answers.ros.org/question/306065/error-on-import-ros-packages/>`_