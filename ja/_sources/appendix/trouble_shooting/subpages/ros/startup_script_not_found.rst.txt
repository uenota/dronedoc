etc/init.d-posix/rcSが見つからない
==============================================================
症状
--------------------------------------------------------------
以下のようにLaunchファイルを起動すると、

.. code-block:: bash

    roslaunch px4 posix_sitl.launch

以下のようなエラーが出る。

.. code-block:: none

    ... logging to /home/uavteam/.ros/log/d9295a54-e8ad-11e8-ae34-40a3cc692f9e/roslaunch-rd-precision5520-8601.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://rd-precision5520:34685/

    SUMMARY
    ========

    PARAMETERS
    * /rosdistro: kinetic
    * /rosversion: 1.12.14
    * /use_sim_time: True

    NODES
    /
        gazebo (gazebo_ros/gzserver)
        gazebo_gui (gazebo_ros/gzclient)
        sitl (px4/px4)
        vehicle_spawn_rd_precision5520_8601_1964229074022197941 (gazebo_ros/spawn_model)

    auto-starting new master
    process[master]: started with pid [8611]
    ROS_MASTER_URI=http://localhost:11311

    setting /run_id to d9295a54-e8ad-11e8-ae34-40a3cc692f9e
    process[rosout-1]: started with pid [8624]
    started core service [/rosout]
    process[sitl-2]: started with pid [8648]
    INFO  [Unknown] Creating symlink /home/uavteam/src/Firmware/ROMFS/px4fmu_common -> /home/uavteam/.ros/etc
    ERROR [Unknown] Error opening startup file, does not exist: etc/init.d-posix/rcS
    process[gazebo-3]: started with pid [8649]
    process[gazebo_gui-4]: started with pid [8654]
    process[vehicle_spawn_rd_precision5520_8601_1964229074022197941-5]: started with pid [8659]
    ================================================================================REQUIRED process [sitl-2] has died!
    process has died [pid 8648, exit code 255, cmd /home/uavteam/src/Firmware/build/posix_sitl_default/bin/px4 /home/uavteam/src/Firmware/ROMFS/px4fmu_common -s etc/init.d-posix/rcS __name:=sitl __log:=/home/uavteam/.ros/log/d9295a54-e8ad-11e8-ae34-40a3cc692f9e/sitl-2.log].
    log file: /home/uavteam/.ros/log/d9295a54-e8ad-11e8-ae34-40a3cc692f9e/sitl-2*.log
    Initiating shutdown!
    ================================================================================
    [vehicle_spawn_rd_precision5520_8601_1964229074022197941-5] killing on exit
    [gazebo_gui-4] killing on exit
    Traceback (most recent call last):
    File "/home/uavteam/catkin_ws/src/gazebo_ros_pkgs/gazebo_ros/scripts/spawn_model", line 32, in <module>
        import tf.transformations as tft
    File "/opt/ros/kinetic/lib/python2.7/dist-packages/tf/__init__.py", line 28, in <module>
    [gazebo-3] killing on exit
    [sitl-2] killing on exit
        from tf2_ros import TransformException as Exception, ConnectivityException, LookupException, ExtrapolationException
    File "/opt/ros/kinetic/lib/python2.7/dist-packages/tf2_ros/__init__.py", line 39, in <module>
        from .buffer_interface import *
    File "/opt/ros/kinetic/lib/python2.7/dist-packages/tf2_ros/buffer_interface.py", line 32, in <module>
        import roslib; roslib.load_manifest('tf2_ros')
    File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslib/launcher.py", line 62, in load_manifest
        sys.path = _generate_python_path(package_name, _rospack) + sys.path
    File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslib/launcher.py", line 93, in _generate_python_path
        m = rospack.get_manifest(pkg)
    File "/usr/lib/python2.7/dist-packages/rospkg/rospack.py", line 167, in get_manifest
        return self._load_manifest(name)
    File "/usr/lib/python2.7/dist-packages/rospkg/rospack.py", line 211, in _load_manifest
        retval = self._manifests[name] = parse_manifest_file(self.get_path(name), self._manifest_name, rospack=self)
    File "/usr/lib/python2.7/dist-packages/rospkg/rospack.py", line 201, in get_path
        self._update_location_cache()
    File "/usr/lib/python2.7/dist-packages/rospkg/rospack.py", line 184, in _update_location_cache
        list_by_path(self._manifest_name, path, cache)
    File "/usr/lib/python2.7/dist-packages/rospkg/rospack.py", line 68, in list_by_path
        is_metapackage = root.find('./export/metapackage') is not None
    KeyboardInterrupt
    [rosout-1] killing on exit
    [master] killing on exit
    shutting down processing monitor...
    ... shutting down processing monitor complete
    done

もしかして
------------------------------------------------------------
``~/.ros/etc/init.d-posix`` 以下にrcSというファイルがありますか？

PX4 SITLシミュレーションでは最初にスタートアップスクリプト（ ``~/.ros/etc/init.d-posix`` ）がロードされます。
普通はセットアップの際に自動で ``~/src/Firmware/ROMFS/px4fmu_common`` から ``~/.ros/etc`` へのシンボリックリンクが作成されますが、何らかの理由で作成されなかったのだと考えられます。

以下のコマンドを使って``~/src/Firmware/ROMFS/px4fmu_common`` から ``~/.ros/etc`` へのシンボリックリンクを作成しましょう。

.. code-block:: bash

    ln -s ~/src/Firmware/ROMFS/px4fmu_common ~/.ros/etc