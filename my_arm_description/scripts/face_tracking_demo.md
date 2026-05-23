Face tracking demo

1) Start the tracker:

   ros2 launch my_arm_description face_tracking_real.launch.py

2) If you see oscillation, reduce gain_pan/gain_tilt or step_limit_deg.

3) Use return_to_center:=true if you want the arm to re-center when face is lost.
