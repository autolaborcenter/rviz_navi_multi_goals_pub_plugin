#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H

//所需要包含的头文件
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>

class QLineEdit;

namespace navi_multi_goals_pub_rviz_plugin {
// 所有的plugin都必须是rviz::Panel的子类
    class TeleopPanel : public rviz::Panel {
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
    Q_OBJECT
    public:
        // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
        TeleopPanel(QWidget *parent = 0);


        // 公共槽.
    public Q_SLOTS:

        // 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher
        void setMaxNumGoal(const QString &maxNumGoal);

        void writePose(geometry_msgs::Pose pose);

        // 内部槽.
    protected Q_SLOTS:

        void updateMaxNumGoal();             // update max number of goal
        void initPoseTable();               // initialize the pose table

        void updatePoseTable();             // update the pose table
        void startNavi();                   // start navigate for the first pose
        void cancelNavi();

        void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose);  //goal count sub callback function

        void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses); //status sub callback function

        void checkCycle();

        void completeNavi();               //after the first pose, continue to navigate the rest of poses
        void cycleNavi();

        bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list);  // check whether arrived the goal

        void startSpin(); // spin for sub
        // 内部变量.
    protected:
        QLineEdit *output_maxNumGoal_editor_;
        QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_cancel_button_;
        QTableWidget *poseArray_table_;
        QCheckBox *cycle_checkbox_;

        QString output_maxNumGoal_;


        // The ROS node handle.
        ros::NodeHandle nh_;
        ros::Publisher goal_pub_, cancel_pub_;
        ros::Subscriber goal_sub_, status_sub_;


        int maxNumGoal_;
        int curGoalIdx_ = 0, cycleCnt_ = 0;
        bool permit_ = false, cycle_ = false;
        geometry_msgs::PoseArray pose_array_;
        actionlib_msgs::GoalID cur_goalid_;

    };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // TELEOP_PANEL_H
