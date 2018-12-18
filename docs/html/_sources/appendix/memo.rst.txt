..
    Lidar liteやopt flowセンサを使うときは、 xxxx_foo.post ファイルを作って、その中にmavlinkで通信するための設定を記述します。
    xxxx_foo.post ファイルは、 xxxx_foo ファイルが読み込まれてから読み込まれる設定ファイルです。
    Lidar Liteやopt flowセンサを使う場合の設定は ~/.ros/etc/init.d-posix/1010_iris_opt_flow.post ファイルが参考になるでしょう。
    `mavlink command <https://dev.px4.io/en/middleware/modules_communication.html#mavlink>`_
    .. code-block:: bash
        mavlink stream -r 10 -s DISTANCE_SENSOR -u $udp_gcs_port_local

    gitコマンドの使い方

        GitHub上のROSパッケージを使う
        ----------------------------------------------
        GitHub上にあるリポジトリは ``git`` コマンドを使ってローカルにクローン（複製）することができます。
        例えば、上記のRotors Simulatorリポジトリであれば、

        .. code-block:: bash

            git clone https://github.com/ethz-asl/rotors_simulator.git

        また、ROSパッケージはワークスペース以下の ``src`` ディレクトリにクローンしてビルドすれば通常のROSパッケージと同様に使用することができます。

        Rotors Simulatorを準備するために以下のコマンドを実行します。

        .. code-block:: bash

            cd ~/catkin_ws/src
            git clone https://github.com/ethz-asl/rotors_simulator.git
            cd ..
            catkin_make

        次のようなエラーが出るときは、

        .. code-block:: none

            Could not find a package configuration file provided by "mav_msgs" with any
            of the following names:

                mav_msgsConfig.cmake
                mav_msgs-config.cmake