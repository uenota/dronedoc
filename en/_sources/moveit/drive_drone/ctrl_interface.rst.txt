コード解説（インターフェース)
---------------------------------------------------------------------------

.. literalinclude:: ../../../include/dronedoc/follow_multi_dof_joint_trajectory_controller_handle.hpp
    :linenos:
    :language: cpp
    :caption: follow_multi_dof_joint_trajectory_controller_handle.hpp

このインターフェースの実装は `FollowJointTrajectory <https://github.com/ros-planning/moveit/blob/f805328aefed52335558c3ea1a72937f235378fd/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>`_ を参考にしています。

.. code-block:: cpp

    #include <moveit_simple_controller_manager/action_based_controller_handle.h>
    #include <moveit_msgs/ExecuteTrajectoryAction.h>

必要なヘッダファイルをインクルードします。
FollowMultiDOFJointTrajectoryインターフェースは、`ExecuteTrajectory <http://docs.ros.org/api/moveit_msgs/html/action/ExecuteTrajectory.html>`_ アクションを使用します。

.. code-block:: cpp

    namespace dronedoc

``dronedoc`` 名前空間を使用します。

.. code-block:: cpp

    class FollowMultiDOFJointTrajectoryControllerHandle
        : public moveit_simple_controller_manager::ActionBasedControllerHandle<moveit_msgs::ExecuteTrajectoryAction>

`ActionBasedControllerHandle <https://github.com/ros-planning/moveit/blob/f805328aefed52335558c3ea1a72937f235378fd/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h>`_ を継承したクラスを定義します。

.. code-block:: cpp

    bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override

動作を実行する際には、MoveIt!に登録されたインターフェースの ``sendTrajectory`` 関数が、`TrajectoryExecutionManagerによってコール <https://github.com/ros-planning/moveit/blob/a1b0efb855af5798d62c4c450e06234abe670bd2/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L470>`_ されます。
このときに経路の情報（ `moveit_msgs/RobotTrajectory <http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html>`_ メッセージ）が与えられるので、これをアクションのゴールとして送信します。

.. code-block:: cpp

    if (!trajectory.joint_trajectory.points.empty())
    {
        ROS_WARN_NAMED("FollowMultiDOFJointTrajectoryController", "%s cannot execute trajectories(trajectory_msgs/JointTrajectory).", name_.c_str());
    }

``joint_trajectory`` フィールドに経由点が格納されている場合は、実行されない旨の警告を表示します。

.. code-block:: cpp

    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory = trajectory;

アクションサーバに送信するためのメッセージを作成します。

.. code-block:: cpp

    controller_action_client_->sendGoal(
        goal, boost::bind(&FollowMultiDOFJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
        boost::bind(&FollowMultiDOFJointTrajectoryControllerHandle::controllerActiveCallback, this),
        boost::bind(&FollowMultiDOFJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));

アクションサーバにゴールを送信します。
同時に、種々のコールバック関数を登録します。

.. code-block:: cpp

    void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const moveit_msgs::ExecuteTrajectoryResultConstPtr& result)
    {
        // Output custom error message for FollowJointTrajectoryResult if necessary
        if (result)
        {
            if(result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                ROS_INFO_NAMED("FollowMultiDOFTrajectoryContoller", "Execution Succeeded.");
            }
            else
            {
                ROS_ERROR("Returned Error Code %d", result->error_code.val);
                ROS_ERROR("For Detailse of Error Code, see moveit_msgs/MoveItErrorCodes.msg");
            }
        }
        else
        {
            ROS_WARN_STREAM("Controller " << name_ << ": no result returned");
        }

        finishControllerExecution(state);
    }

アクションサーバが実行を終了した場合に呼ばれます。
エラーが発生した場合はエラーメッセージを、成功したら成功したことを通知するメッセージを表示します。