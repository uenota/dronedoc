コード解説（マネージャ)
---------------------------------------------------------------------------

.. literalinclude:: ../../../src/moveit_multi_dof_controller_manager.cpp
    :linenos:
    :language: cpp
    :caption: moveit_multi_dof_controller_manager.cpp

このマネージャの実装は `MoveItSimpleControllerManager <https://github.com/ros-planning/moveit/blob/f805328aefed52335558c3ea1a72937f235378fd/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp>`_ を参考にしています。

.. code-block:: cpp

    #include <ros/ros.h>
    #include <moveit/controller_manager/controller_manager.h>
    #include <dronedoc/follow_multi_dof_joint_trajectory_controller_handle.hpp>
    #include <sensor_msgs/JointState.h>
    #include <pluginlib/class_list_macros.hpp>

必要なヘッダファイルをインクルードします。
``follow_multi_dof_joint_trajectory_controller_handle.hpp`` ヘッダは、 ``FollowMultiDOFJointTrajectory`` インターフェースを定義しています。
また、MoveIt!のコントローラマネージャは、`pluginlib <http://wiki.ros.org/pluginlib>`_ を用いてプラグインとして登録して使用するので、pluginlibのヘッダをインクルードします。

.. code-block:: cpp

    namespace dronedoc

``dronedoc`` 名前空間を使用します。

.. code-block:: cpp

    class MoveItMultiDOFControllerManager : public moveit_controller_manager::MoveItControllerManager

プラグインとして利用するためには、特定のベースクラスを継承したクラスを定義する必要があります。
コントローラマネージャをMoveIt!のプラグインとして登録するためには `moveit_controller_manager::MoveItControllerManager <http://docs.ros.org/jade/api/moveit_core/html/classmoveit__controller__manager_1_1MoveItControllerManager.html>`_ クラスを継承して、メソッドをオーバーライドする必要があります。

プラグインの作成の詳細については、`Writing and Using a Simple Plugin <http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin>`_ を参照してください。

.. code-block:: cpp

    if(controller_list.size() > 1)
    {
      ROS_ERROR("This controller manager expects only one controller.");
      return;
    }

このマネージャでは複数のコントローラを使用することは想定していないので、 ``controller_list`` パラメータに複数のコントローラが指定されている場合にはエラーメッセージを表示します。

.. code-block:: cpp

    if (!controller_list[0].hasMember("name") || !controller_list[0].hasMember("joints"))
    {
        ROS_ERROR_STREAM_NAMED("manager", "Name and joints must be specifed for each controller");
        return;
    }

``controller_list`` パラメータに必要なフィールドが設定されていない場合にはエラーメッセージを表示します。

.. code-block:: cpp

    moveit_simple_controller_manager::ActionBasedControllerHandleBasePtr new_handle;
    if (type == "FollowMultiDOFJointTrajectory")
    {
        new_handle.reset(new FollowMultiDOFJointTrajectoryControllerHandle(name, action_ns));
        if (static_cast<FollowMultiDOFJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
        {
            ROS_INFO_STREAM_NAMED("manager", "Added FollowJointTrajectory controller for " << name);
            controller_ = new_handle;

コントローラインターフェースのタイプが ``FollowMultiDOFJointTrajectory`` には新しくコントローラインターフェースを登録します。

.. code-block:: cpp

    moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
    {
        return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(controller_);
    }

コントローラインターフェースへのポインタを返す関数です。
コントローラインターフェースへのポインタを親クラスへのポインタへキャストして返しています。

.. code-block:: cpp

    PLUGINLIB_EXPORT_CLASS(dronedoc::MoveItMultiDOFControllerManager,
                           moveit_controller_manager::MoveItControllerManager);

``PLUGINLIB_EXPORT_CLASS`` マクロを使ってクラスをプラグインとして登録します。
第一引数には定義したクラスを、第二引数にはベースクラスを与えます。