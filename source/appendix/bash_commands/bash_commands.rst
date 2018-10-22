====================
基本的なBashコマンド
====================

はじめに
========
ここでは基本的なbashコマンドとbash関連の設定等を解説します。

Bashって?
==========
TODO

基本的なコマンド
===============
TODO: コマンドを分類する

ls
----
``ls`` はファイルやディレクトリの内容を表示するコマンドです。

使い方
^^^^^^
``ls`` コマンド単体で実行するとカレントディレクトリの内容を表示します。

.. code-block:: bash

    ls

特定のパスを指定するとそのディレクトリの内容を表示します。

.. code-block:: bash

    ls ~/catkin_ws/src

``-a`` オプションをつけると対象のディレクトリのすべてのフォルダ、ファイル（隠しフォルダ等を含む）を表示します。

.. code-block:: bash

    ls -a

``-l`` オプションをつけると対象のディレクトリの内容をより詳細に表示することができます。

.. code-block:: bash
    
    ls -l

これらのオプションは同時に指定することもできます。

.. code-block:: bash

    ls -la

cd
----
TODO

touch
------
TODO

mkdir
------
TODO

echo
------
TODO

rm
----
TODO

cp
----
TODO

mv
----
TODO

ssh
----
TODO

scp
----
TODO

wget
----
TODO

curl
----
TODO

source
-------
TODO

export
--------
TODO

cat
----
TODO

less
------
TODO

リダイレクト
============
TODO

.bashrc
============
.bashrcはBashの設定スクリプトです。

Tips
-----

set -o noclobber
^^^^^^^^^^^^^^^^^
TODO

alias
^^^^^^
TODO

参考
==========
linuxのあれ（TODO）