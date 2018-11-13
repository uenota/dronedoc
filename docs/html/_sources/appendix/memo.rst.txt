..
    Lidar liteやopt flowセンサを使うときは、 xxxx_foo.post ファイルを作って、その中にmavlinkで通信するための設定を記述します。
    xxxx_foo.post ファイルは、 xxxx_foo ファイルが読み込まれてから読み込まれる設定ファイルです。
    Lidar Liteやopt flowセンサを使う場合の設定は ~/.ros/etc/init.d-posix/1010_iris_opt_flow.post ファイルが参考になるでしょう。
    `mavlink command <https://dev.px4.io/en/middleware/modules_communication.html#mavlink>`_
    .. code-block:: bash
        mavlink stream -r 10 -s DISTANCE_SENSOR -u $udp_gcs_port_local
