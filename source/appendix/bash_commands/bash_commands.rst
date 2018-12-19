基本的なBashの使い方
****************************************

はじめに
========================================
ここでは基本的なbashコマンドとbash関連の設定等を解説します。

大抵の場合 ``コマンド名 linux`` で検索すると出てきます。
ここに書いていないコマンドがあった場合は調べながら試してみるとよいでしょう。
Linuxコマンドに関しては調べながら試してみるのが一番だと思います。

基本的なコマンド
========================================
.. TODO: コマンドを分類する

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

Git
==========

Gitはバージョン管理ソフトウェアの一種で、インストール済みであれば ``git`` コマンドから利用することができます。

gitコマンドの使い方
----------------------------------------------
GitHub上にあるリポジトリは ``git`` コマンドを使ってローカルにクローン（複製）することができます。

例えば、このチュートリアルのリポジトリであれば、

.. code-block:: bash

    git clone https://github.com/uenota/dronedoc.git

GitHub上のROSパッケージを使う
----------------------------------------------
ROSパッケージはワークスペース以下の ``src`` ディレクトリにクローンしてビルドすれば通常のROSパッケージと同様に使用することができます。

.. code-block:: bash

    cd ~/catkin_ws/src
    git clone https://github.com/uenota/dronedoc.git
    cd ..
    catkin_make


参考
==========
