コード解説（コントローラ)
---------------------------------------------------------------------------

.. literalinclude:: ../../../src/drone_controller.cpp
    :linenos:
    :language: cpp
    :caption: drone_controller.cpp

.. code-block:: cpp

    #include <vector>
    #include <array>
    #include <string>
    #include <cmath>

    #include <ros/ros.h>

    #include <geometry_msgs/Transform.h>
    #include <geometry_msgs/PoseStamped.h>

    #include <moveit_msgs/ExecuteTrajectoryAction.h>
    #include <moveit_msgs/RobotTrajectory.h>

    #include <mavros_msgs/State.h>

    #include <trajectory_msgs/MultiDOFJointTrajectory.h>
    #include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

    #include <actionlib/server/simple_action_server.h>

必要なヘッダファイルをインクルードします。
コントローラ内でアクションサーバを実装するので、actionlibのヘッダファイルもインクルードしています。

.. code-block:: cpp

    //! Action name of ExecuteTrajectory
    std::string action_name_;
    //! Server of ExecuteTrajectoryAction
    actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction> as_;
    //! Feedback message of ExecuteTrajectoryAction
    moveit_msgs::ExecuteTrajectoryFeedback feedback_;
    //! Result message of ExecuteTrajectoryAction
    moveit_msgs::ExecuteTrajectoryResult result_;

アクションサーバとサーバ名、サーバが使用するフィードバックとリザルトのメッセージを初期化します。
FollowMultiDOFJointTrajectoryインターフェースは、`ExecuteTrajectory <http://docs.ros.org/api/moveit_msgs/html/action/ExecuteTrajectory.html>`_ アクションを使用するので、アクションサーバを宣言する際のテンプレート引数として与えています。

.. code-block:: cpp

    //! Position command publisher
    ros::Publisher cmd_pos_pub_;

位置指令を送信するためのパブリッシャです。

.. code-block:: cpp

    //! Subscriber of local position
    ros::Subscriber local_pos_sub_;
    //! Subscriber of UAV state
    ros::Subscriber state_sub_;
    //! Current pose of UAV
    geometry_msgs::PoseStamped current_pose_;
    //! Current state of UAV
    mavros_msgs::State current_state_;

ドローンの現在位置と状態を取得するためのパブリッシャとそれを格納するための変数です。

.. code-block:: cpp

    DroneController(std::string action_name) :
        action_name_(action_name),
        as_(nh_, action_name, boost::bind(&DroneController::executeCb, this, _1), false)
    {
        as_.start();

        // Position command publisher setup
        std::string cmd_pos_topic;
        nh_.param<std::string>("mavros_setpoint_topic", cmd_pos_topic, "/mavros/setpoint_position/local");
        cmd_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cmd_pos_topic, 10);

        // Local position subscriber setup
        std::string local_pos_topic;
        nh_.param<std::string>("mavros_localpos_topic", local_pos_topic, "/mavros/local_position/pose");
        auto local_pos_cb = boost::bind(&DroneController::localPosCb, this, _1);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(local_pos_topic, 10, local_pos_cb);

        // UAV state subscriber setup
        std::string state_topic;
        nh_.param<std::string>("mavros_state_topic", state_topic, "/mavros/state");
        auto state_cb = boost::bind(&DroneController::stateCb, this, _1);
        state_sub_ = nh_.subscribe<mavros_msgs::State>(state_topic, 10, state_cb);
        ROS_INFO("Action server initialized.");
    };

``DroneController`` クラスのコンストラクタです。
内部ではパブリッシャとサブスクライバ、アクションサーバの初期化を行っています。

使用するトピック名は、ROSの `パラメータサーバから取得 <http://wiki.ros.org/roscpp/Overview/Parameter%20Server>`_ します。

サブスクライバとアクションサーバはコールバック関数を使用するので、メソッドをコールバック関数に指定するためにboost::functionオブジェクトを受け取る `関数 <http://docs.ros.org/jade/api/roscpp/html/classros_1_1NodeHandle.html#a93b0fe95db250c45fdfbc5fd9f4c0159>`_ と、`コンストラクタ <http://docs.ros.org/melodic/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a0f047d3a7474d3c2c1d3a0c7ef6adf16>`_ をそれぞれ使用して初期化しています。

.. code-block:: cpp

    void executeCb(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal)

アクションのコールバック関数です。
アクションゴールが届いたタイミングで実行されます。

.. code-block:: cpp

    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory;
    trajectory = goal->trajectory.multi_dof_joint_trajectory.points;

アクションゴールは、`moveit_msgs/RobotTrajectory <http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html>`_ 型のメッセージなので、そこから経由点の配列を取り出しています。

.. code-block:: cpp

    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() and not current_state_.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connection established.");

ドローンに接続するまで待機します。

.. code-block:: cpp

    for(int i=0; i<10; ++i)
    {
        cmd_pos_pub_.publish(getPoseFromTrajectory(trajectory.front()));
    }

OFFBOARDモードに変更するためには予め ``setpoint_position/local`` トピックにメッセージがパブリッシュされている必要があるので、予めパブリッシュしておきます。

.. code-block:: cpp

    for(int i=0; i < trajectory.size()-1; ++i)
    {
        ROS_INFO("Moving to waypoint No. %d", i);

各経由点を処理していきます。

.. code-block:: cpp

    feedback_.state = std::to_string(i);
    as_.publishFeedback(feedback_);

現在処理している経由点の番号をアクションのフィードバックとして登録します。

.. code-block:: cpp

    geometry_msgs::PoseStamped start = getPoseFromTrajectory(trajectory.at(i));
    geometry_msgs::PoseStamped goal = getPoseFromTrajectory(trajectory.at(i+1));

``trajectory_msgs/MultiDOFJointTrajectoryPoint`` メッセージを、``geometry_msgs/PoseStamped`` メッセージに変換します。

.. code-block:: cpp

    std::vector<geometry_msgs::PoseStamped> path = getBilinearPath(start, goal);

`バイリニア補間 <https://en.wikipedia.org/wiki/Bilinear_interpolation>`_ を行って経由点間の点の配列を取得します。

.. code-block:: cpp

    for(auto pose: path)

取得した配列の各点について処理を行います。

.. code-block:: cpp

    while(not isGoalReached(pose) and not as_.isPreemptRequested())
    {
        cmd_pos_pub_.publish(pose);
        rate.sleep();
    }

ドローンが次の点に到達するか、アクションを中止するリクエストが届くまで目標位置を送信し続けます。

.. code-block:: cpp

    if(as_.isPreemptRequested() or not ros::ok())
    {
        result_.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
        as_.setPreempted();
        ROS_INFO("Action preempted.");
        break;
    }

アクションの中止がリクエストされた時には、状態を反映してアクションサーバを終了します。

.. code-block:: cpp

    result_.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    as_.setSucceeded(result_);
    ROS_INFO("Action completed.");

ゴールまで移動したので結果を反映してクライアントへと送信します。

.. code-block:: cpp

    std::vector<geometry_msgs::PoseStamped> getBilinearPath(const geometry_msgs::PoseStamped &start,
                                                            const geometry_msgs::PoseStamped &goal,
                                                            const double step=0.05)
    {
        std::vector<geometry_msgs::PoseStamped> bilinear_path;

        // Store x-y and x-z coordinates of start point
        std::array<double, 2> start_xy = {start.pose.position.x, start.pose.position.y};
        std::array<double, 2> start_xz = {start.pose.position.x, start.pose.position.z};
        // Store x-y and x-z coordinates of goal point
        std::array<double, 2> goal_xy = {goal.pose.position.x, goal.pose.position.y};
        std::array<double, 2> goal_xz = {goal.pose.position.x, goal.pose.position.z};

        // x-y and x-z coordinates of interpolated points
        std::array<std::vector<double>, 2> points_xy = linearInterp(start_xy, goal_xy, step);
        std::array<std::vector<double>, 2> points_xz = linearInterp(start_xz, goal_xz, step);

        // Number of generated points by interpolation
        int num_points = points_xy.at(0).size();

        try
        {
            // Generate PoseStamped message from std::array
            for(int i=0; i<num_points; ++i)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.orientation = start.pose.orientation;
                pose.pose.position.x = points_xy.front().at(i);
                pose.pose.position.y = points_xy.back().at(i);
                pose.pose.position.z = points_xz.back().at(i);

                bilinear_path.push_back(pose);
            }
        }
        catch (std::out_of_range &ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        return bilinear_path;
    }

``geometry_msgs/PoseStamped`` 型で表されたスタート地点とゴール地点の座標を受け取って、バイリニア補間を行い、補間された点の配列を返す関数です。

.. code-block:: cpp

    std::array<std::vector<double>, 2> linearInterp(const std::array<double, 2> &p1,
                                                    const std::array<double, 2> &p2, const double step)
    {
        // Gradient
        double a = (p1.at(1) - p2.at(1)) / (p1.at(0) - p2.at(0));
        // Intercept
        double b = p1.at(1) - a*p1.at(0);

        // Number of steps
        int num_steps = std::floor((p2.at(0) - p1.at(0))/step);

        // Initialize container for interpolated points
        std::vector<double> points_x(num_steps+1);

        // Set interpolated points
        points_x.front() = p1.at(0);
        for(int i=1; i<num_steps; ++i)
        {
            points_x.at(i) = step * i + p1.at(0);
        }
        points_x.back() = p2.at(0);

        // Initialize container for interpolated points
        std::vector<double> points_y(num_steps+1);

        // Set interpolated points
        points_y.front() = p1.at(1);
        for(int i=1; i<num_steps; ++i)
        {
            points_y.at(i) = a*(p1.at(0) + i*step) + b;
        }
        points_y.back() = p2.at(1);

        // Initialize container for vector of points
        std::array<std::vector<double>, 2> points;
        points.front() = points_x;
        points.back() = points_y;

        return points;
    }

二点の二次元座標を受け取って線形補間を行い、補間された点の座標の配列を返す関数です。

.. code-block:: cpp

    geometry_msgs::PoseStamped getPoseFromTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_pt)
    {
        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = trajectory_pt.transforms.front().translation.x;
        pose.pose.position.y = trajectory_pt.transforms.front().translation.y;
        pose.pose.position.z = trajectory_pt.transforms.front().translation.z;
        pose.pose.orientation = trajectory_pt.transforms.front().rotation;

        return pose;
    }

``trajectory_msgs/MultiDOFJointTrajectoryPoint`` メッセージを ``geometry_msgs/PoseStamped`` メッセージへ変更して返す関数です。

.. code-block:: bash

    inline bool isGoalReached(const geometry_msgs::PoseStamped &goal, const double tolerance=0.1)
    {
        double error = std::sqrt(std::pow(goal.pose.position.x - current_pose_.pose.position.x, 2)
                                + std::pow(goal.pose.position.y - current_pose_.pose.position.y, 2)
                                + std::pow(goal.pose.position.z - current_pose_.pose.position.z, 2));
        return error < tolerance ? true : false;
    }

ドローンが目標位置に到達したかどうかを判断する関数です。
デフォルトでは、目標位置を中心とした0.1 m以内の球体内にドローンが存在する場合に目標位置へ到達したと判定します。

.. code-block:: cpp

    void localPosCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
    }

    void stateCb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state_ = *msg;
    }

ドローンの位置と状態のコールバック関数です。
それぞれメンバ変数に取得したメッセージを格納しています。

.. code-block:: cpp

    int main(int argv, char **argc)
    {
        ros::init(argv, argc, "drone_controller");

        DroneController controller("iris_group_controller/follow_multi_dof_joint_trajectory");
        ros::spin();

        return 0;
    }

``drone_controller`` ノード内でコントローラを初期化します。
FollowMultiDOFJointTrajectoryインターフェースが利用するメッセージは、``iris_group_controller/follow_multi_dof_joint_trajectory`` 以下でやりとりされるので、これをコントローラのコンストラクタへ与えます。