:tocdepth: 1

******************************************************
Octomapを使って3Dマップを作る
******************************************************
以下の内容は、 `OctoMap 3D Models with ROS/Gazebo - PX4 Developer Guide <https://dev.px4.io/en/simulation/gazebo_octomap.html>`_ を参考にしています。

Rotors Simulatorを準備する
=============================================
このチュートリアルでは、 `Rotors Simulator <https://github.com/ethz-asl/rotors_simulator>`_ を使用します。

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