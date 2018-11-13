TFの向きが反対になっている
==================================================================================
症状
----------------------------------------------------------------------------------
- TFの値の正負が逆転している
- Rvizで見た時にTFが原点に対して反対に位置している

もしかして？
----------------------------------------------------------------------------------
``lookupTransform`` のフレームの順序を変えるとなおるかもしれません。

``lookupTransform`` 関数は、 ``target_frame`` に指定したフレームから ``source_frame`` へのTFを ``transform`` へ格納します。

.. code-block:: cpp

  Transformer::lookupTransform(const std::string& target_frame,
                               const std::string& source_frame,
                               const ros::Time& time,
                               StampedTransform& transform)

参考
----------------------------------------------------------------------------------
`Is arguments of lookupTransform opposite? <https://github.com/ros/geometry/issues/108>`_