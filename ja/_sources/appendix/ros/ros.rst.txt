ROSの基本
************************************

ROSコマンド
====================================
.. TODO:

`ROS Command-line tools <http://wiki.ros.org/ROS/CommandLineTools>`_

パブリッシャとサブスクライバ
====================================
ノード同士はトピックを通じてメッセージを送受信することでやり取りしています。
ノードがトピックにメッセージを送信することをパブリッシュするといい、
データをパブリッシュするノードをパブリッシャといいます。
また、ノードがトピックからメッセージを受け取ることをサブスクライブするといい、
データをサブスクライブするノードをサブスクライバといいます。

.. image:: imgs/topic.png

サーバとクライアント
====================================
`サービス <http://wiki.ros.org/Services>`_ は、2つのノードが1対1でやり取りするときに使われる仕組みです。
サービスの呼び出しでは、クライアントと呼ばれるノードがもう一方のノード（サーバ）に対してリクエストを送信し、
サーバが何らかの処理を行った後にクライアントに対してレスポンスを送信します。

``rosservice`` コマンド
------------------------------------
`rosservice <http://wiki.ros.org/rosservice>`_ コマンドを使えばROSサービスをコールしたり、その情報を調べることができます。

サービスファイル(.srv)
------------------------------------
ノードが提供するサービスについての決まり事が書いてあるファイルです。
具体的には、サービスのリクエストとレスポンスのデータ型が記載されています。
`srv`フォルダ以下に格納されます。
サービスファイルの文法などは `srv - ROS.org <http://wiki.ros.org/srv>`_ を参照してください。

以下はturtlesimのSpawn.srvファイルです。
turtleを新しく配置するときに使われるサービスです。
点線で区切られた上の部分がリクエスト、下の部分がレスポンスです。

turtleを配置する位置と角度、名前をリクエストとして与えると、
サーバがturtleを新しく配置して、配置したturtleの名前をレスポンスとして返してきます。

.. code-block:: guess

  float32 x
  float32 y
  float32 theta
  string name # Optional.  A unique name will be created and returned if this is empty
  ---
  string name

メッセージファイルと同様、 ``#`` 以降はコメントと判断されます。


TF
====================================
.. TODO:

`tf/Tutorial - ROS Wiki <http://wiki.ros.org/ja/tf/Tutorials>`_

パラメータ
====================================
ROSノードは、再利用性を高めるためにパラメータを用いてその動作を変更できるようにしてあります。

ノード内でパラメータを利用する方法については、 `roscpp <http://wiki.ros.org/roscpp/Overview/Parameter%20Server>`_ と `rospy <http://wiki.ros.org/rospy/Overview/Parameter%20Server>`_ のドキュメンテーションを参考にしてください。

``rosparam`` コマンド
------------------------------------
ノードのパラメータは ``rosparam`` コマンドを使って確認したり設定したりすることができます。
rosparamコマンドについては `rosparam - ROS Wiki <http://wiki.ros.org/rosparam>`_ を見てください。

また、Launchファイルからノードを起動した時は、以下のようにパラメータ一覧が表示されます。

.. image:: imgs/param_launch.png

参考
------------------------------------
`Parameter Server - ROS Wiki <http://wiki.ros.org/ja/Parameter%20Server>`_

各種ファイルの書き方
====================================

Launchファイル
------------------------------------
.. TODO:

`roslaunch/XML <http://wiki.ros.org/roslaunch/XML>`_

package.xml
------------------------------------
.. TODO:

`catkin/package.xml <http://wiki.ros.org/catkin/package.xml>`_

CMakeLists.txt
------------------------------------
.. TODO:

`catkin/cmake <http://wiki.ros.org/catkin/CMakeLists.txt>`_