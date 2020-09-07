# scripts

This is a backup from my company account, which is my memory of 2 years of work.

***

1. added butter worth digital filter:
USAGE: use pyulog log.ulog and add actuator_controls_0_0.csv path to the script

2. added setpoint&heading:
Dependencies: python2 && dronekit && mavlink
NOTE: 注意不要将QGC打开，否则会有warning，特别烦人。 可能执行的时候会卡在串口连接那块。

3. added mavlink_api:
将所有的mavlink相关的东西放在mavlink_api文件夹内， 里面有一个apm可以用的mavlink api还有一个apm的example， 可以走正方形。

4. 新增了px4的mavros控制指令，分为不能退出offboard模式的不安全版本以及一个可以退出offboard的指GPS点飞行脚本。

5. 新增px4通过csv读取 起飞，飞行到多个GPS点，降落操作

6. 1129成功gps指点实验， 新组装的中小型飞机，起飞并飞到GPS点，复现不稳定，起飞会漂移，每次试验需要重启飞控。

7. px4 mavros api now works perfectly.

8. added Cooleye SDK that removed requirements for opencv_contrib. Follow cooleye github to compile

9. added realtime pose sub and execute for px4 based on mavros and python

10. added script to extract image from a bag on a single topic; usage: python bag_to_images.py '2019-01-09-15-04-10.bag' 'temp/left' /mynteye/right_rect/image_rect

11. added camera_ros: convert cv images to ros node

12. added stere_calibration and stereo_dense_reconstruction

13. added a workding and modified px4_mavros_commander

14. added dockerfiles needed for building different docker images

15. added GAAS_tutorials

16. added obstacle map for indoor obstacle avoidance and 2D graphy navigator

17. add RRT star

18. add obstacle avoidance for gazebo and tx2 version using random sampling method.


