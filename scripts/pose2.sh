rosparam set /intent 0
rosparam set /target 0
rostopic pub /object_array hirop_msgs/ObjectArray "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
objects:
- name: ''
  detector: ''
  pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'base_link'
    pose:
      position:
        x: 0.85
        y: 0
        z: 0.25
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1"