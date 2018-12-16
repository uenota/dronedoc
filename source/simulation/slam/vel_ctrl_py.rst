px4_vel_controller.py
===============================================================
.. literalinclude:: ../../../script/px4_vel_controller.py
  :linenos:
  :language: python

解説
===============================================================

.. code-block:: python

    class PX4VelocityController:

グローバル変数を使う代わりに、メンバ変数を使うので、速度指令をパブリッシュするクラスを宣言します。

.. code-block:: python

    def __init__(self):
        self.origin_frame = rospy.get_param("origin_frame",
                                            "map")
        self.vehicle_frame = rospy.get_param("vehicle_frame",
                                            "base_link")
        self.move_base_topic = rospy.get_param("move_base_topic",
                                            "/cmd_vel")
        self.mavros_topic = rospy.get_param("mavros_topic",
                                            "/mavros/setpoint_velocity/cmd_vel")

``PX4VelocityController`` クラスのコンストラクタです。
コンストラクタ内部では、ROSパラメータを使ってフレーム名やトピック名などの変数を初期化しています。

.. code-block:: python

    self.transformer = tf.TransformListener()

ロボットのベースフレームへのTFを参照するためのリスナーを初期化しています。

.. code-block:: python

    self.cmd_pub = rospy.Publisher(self.mavros_topic,
                                    TwistStamped,
                                    queue_size=10)
    self.cmd_sub = rospy.Subscriber(self.move_base_topic,
                                    Twist, self.vel_cmd_cb)

それぞれ、mavrosへ速度指令を送るためのパブリッシャと、move_baseからの速度指令を受け取るためのサブスクライバです。

.. code-block:: python

    def vel_cmd_cb(self, msg):
        linear_vel = Vector3Stamped()
        linear_vel.header.frame_id = self.vehicle_frame
        linear_vel.header.stamp = rospy.Time().now()

        linear_vel.vector.x = msg.linear.x
        linear_vel.vector.y = msg.linear.y
        linear_vel.vector.z = 0

move_baseから受け取った速度指令を処理するためのコールバック関数です。
まずはじめに、受け取ったメッセージの値を ``Vector3Stamped`` 型の変数に格納しています。
この変数に対して座標変換を行うことで、mavrosに送信する速度指令を得ます。

.. code-block:: python

        try:
            self.transformer.waitForTransform(self.vehicle_frame, self.origin_frame, rospy.Time().now(), rospy.Duration(0.05))
            linear_vel = self.transformer.transformVector3(self.origin_frame, linear_vel)

``waitForTransform`` 関数で、ロボットのベースフレームへのTFが得られるまで待機します。
第四引数に与えられた時間が経過してもTFが得られなかった場合は、 ``tf.Exception`` 例外を送出します。

TFが届いたあとは、 ``transformVector3`` 関数でmove_baseからの速度指令の座標変換を行います。

.. code-block:: python

        except tf.Exception as e:
            rospy.logerr(e)
            return

``waitForTransform`` 関数は ``tf.Exception`` 例外を、 ``transformVector3`` 関数は ``tf.LookupException`` 、 ``tf.ConnectivityException`` 、 ``tf.ExtrapolationException`` の3つの例外を送出する可能性があるので、 ``try-except`` 節で例外を補足する必要があります。
``tf.LookupException`` 、 ``tf.ConnectivityException`` 、 ``tf.ExtrapolationException`` の3つの例外は ``tf.Exception`` 例外のサブクラスなので、このように記述すれば全ての例外を捕捉できます。
例外を補足した場合は、エラーメッセージを出力して、関数から抜け出します。

.. code-block:: python

        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time().now()
        vel_cmd.header.frame_id = self.origin_frame

        vel_cmd.twist.linear.x = linear_vel.vector.x
        vel_cmd.twist.linear.y = linear_vel.vector.y
        vel_cmd.twist.linear.z = linear_vel.vector.z
        vel_cmd.twist.angular.z = msg.angular.z

        self.cmd_pub.publish(vel_cmd)

mavrosに送信する速度指令のメッセージを作成し、パブリッシュします。

.. code-block:: python

    if __name__ == "__main__":
        rospy.init_node("px4_vel_controller")
        rate = rospy.Rate(20)
        PX4VelocityController()
        rospy.spin()

``PX4VelocityController`` クラスのインスタンスを作成し、ノードを実行します。