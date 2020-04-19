rosparam set /intent 1
rosparam set /target 2
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
        x: 0.418
        y: -0.65
        z: 0.38
      orientation:
        x: 0.0
        y: 0.0
        z: -0.706825
        w: 0.707388"