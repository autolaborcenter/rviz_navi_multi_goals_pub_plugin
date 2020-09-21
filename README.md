# navi_multi_goals_pub_rviz_plugin

此插件是配合ROS-RViz-Navigation来使用的。

主要目的是解决Navigation中只能设置一个目标点，而不是多个。

操作步骤
----
下载
```
git clone https://github.com/RYOOSS/navi_multi_goals_pub_rviz_plugin.git
```
编译后更改*.rviz文件
 ``` 
 Tools:
    - Class: rviz/SetGoal
     Topic: /move_base_simple/goal_temp 
```
启动rviz后，在Rviz的Display中add一个Marker

RViz的左上角依次点击Panels->Add New Panel-> navi_multi_goals_pub_rviz_plugin->MultiNaviGoalsPanel

填写目标最大数量后，确定。

点击ToolBar上的2D Nav Goal后，依次在Map中设置导航目标点，目标点个数不能超过填写的目标最大数量。

点击开始导航，导航开始。

若勾选循环，导航至最后一个目标点后，将重新导航至第一个目标点。

若点击取消，将取消当前目标点导航。

若点击重置，将清空当前所有目标点。

![ryoo rviz](https://github.com/RYOOSS/navi_multi_goals_pub_rviz_plugin/blob/master/image/rviz_screenshot_2020_09_21-12_17_49.png)

