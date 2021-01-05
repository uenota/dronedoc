odom_publisher.py
============================================

.. literalinclude:: ../../script/odom_publisher.py
  :linenos:
  :language: python

解説
============================================
.. code-block:: python

  #!/usr/bin/env python

ROSノードとして実行するために必要な記述です。
詳しくは""python shebang"などで検索してください。

.. code-block:: python

  # Callback for local position
  local_pos = PoseStamped()
  def local_pos_cb(msg):
      # Need global declaration in order to assign global variable
      global local_pos
      local_pos = msg

  # Callback for local velocity
  local_vel = TwistStamped()
  def local_vel_cb(msg):
      # Need global declaration in order to assign global variable
      global local_vel
      local_vel = msg

``mavros/local_position/pose`` トピックと ``mavros/local_position/velocity`` トピックのサブスクライバです。
どちらもグローバル変数に値を格納するだけの関数です。

.. code-block:: python

  odom_quat = tf.transformations.quaternion_from_euler(pose.orientation.x,
                                                        pose.orientation.y,
                                                        pose.orientation.z)

オイラー角からクォータニオンへの変換を行っています。

.. code-block:: python

  odom = Odometry()
  odom.header.stamp = rospy.Time.now()
  # Global frame is "odom"
  odom.header.frame_id = "odom"

  odom.pose.pose.position.x = pose.position.x
  odom.pose.pose.position.y = pose.position.y
  odom.pose.pose.position.z = 0
  odom.pose.pose.orientation = odom_quat

  # Set robot's base frame as odometry child frame
  odom.child_frame_id = "base_link"
  odom.twist.twist.linear.x = twist.linear.x
  odom.twist.twist.linear.y = twist.linear.y
  odom.twist.twist.angular.z = twist.angular.z

  odom_pub.publish(odom)

``Odometry`` 型の変数を作成し、それぞれのフィールドに姿勢や速度などの値を格納しています。
今回はグローバル座標は ``odom`` で、ロボットのベース座標は ``base_link`` になっています。

.. code-block:: python

  if __name__ == "__main__":
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass

一行目は、プログラムがトップレベルスクリプトとして実行された時のみ実行されるようにするための記述です。
``try-except`` 節は、Ctrl-Cなどでノードが停止された時に送出される ``ROSInterruptException`` 例外をキャッチするための部分です。