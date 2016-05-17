# ROS Figaro 

1. Install `ros-indigo-rosjava`
2. Purge `ros-indigo-rosjava-messages` as it is outdated
3. Move to the root of the desired ROS workspace and source it
4. Run `genjava_message_artifacts` in order to generate all java messages
5. Run `catkin_make` to build the `ros_figaro` messges
6. Install `sbt` by following the instructions [here](http://www.scala-sbt.org/0.13/docs/Installing-sbt-on-Linux.html)
7. export the `ROS_IP` so that rosjava can run
8. Run `sbt "runMain org.ros.ros_figaro.Regression"` from the base directory in order to run the regression example

## Troubleshooting

In case of mismatching versions of the rosjava messges run
```
rm -rf ~/.sbt/ ~/.ivy2/
```
clean the catkin workspace (assuming it is called `ros_ws`)
```
cd ros_ws
rm -rf devel build
```
and the sbt files
```
cd ros_ws/src/ros_figaro
rm -rf target project
```
After that repeat steps 3-8 from above.

