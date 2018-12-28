:tocdepth: 1

QGroundControl
=============================================================

`QGroundControl <http://qgroundcontrol.com/>`_ (QGC) はPX4もしくは、ArduPilotファームウェアで動作するドローン用のグラウンドコントロールステーションです。

セットアップ
-------------------------------------------------------------
QGCの公式サイトの `インストールガイド <https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu-linux>`_ からQGroundControl.AppImageをダウンロードします。

ダウンロードしたQGroundControl.AppImageに実行権限を与えます。

.. code-block:: bash

    chmod +x QGroundControl.AppImage

これでセットアップは完了です。
実行するには、アイコンをダブルクリックするか、ターミナルから以下のコマンドで起動します。

.. code-block:: bash

    ./QGroundControl.AppImage

.. image:: imgs/qgc.jpg

基本的な使い方に関しては、`QGroundControl User Guide <https://docs.qgroundcontrol.com/en/>`_ を参照してください。