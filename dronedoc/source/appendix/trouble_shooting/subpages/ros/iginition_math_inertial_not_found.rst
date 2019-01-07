make posix_sitl_default gazeboでignition/math/Inertial.hhが見つからない
====================================================================================

症状
-----------------------------------------------------------------
Firmwareディレクトリで

.. code-block:: bash

    `make posix_sitl_default gazebo`

を実行すると次のようなエラーが出る。

.. code-block:: bash

    `/usr/include/gazebo-7/gazebo/msgs/msgs.hh:24:37: fatal error: ignition/math/Inertial.hh: そのようなファイルやディレクトリはありません`


もしかして
-----------------------------------------------------------------
libignition-math2-devがインストールされていないかもしれません。
次のコマンドを試してみてください。

.. code-block:: bash

    sudo apt-get install libignition-math2-dev

参考
-----------------------------------------------------------------
`Error with make posix_sitl_default gazebo with gazebo7 <https://github.com/PX4/Firmware/issues/5773>`_