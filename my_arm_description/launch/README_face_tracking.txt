face_tracking_real.launch.py

Parameters:
- image_topic: camera image topic
- image_encoding: bgr8 or rgb8
- servo_port: USB serial port
- pan_id / tilt_id: servo IDs to control
- gain_pan / gain_tilt: proportional gains
- step_limit_deg: per-update max change
- update_rate: control rate (Hz)
- return_to_center: re-center when face lost
