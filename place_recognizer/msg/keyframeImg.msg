int32 id			#keyframe id
float64 time			#timestamp

# camera parameter (fx fy cx cy), width, height
# will never change, but required for display.
float32 fx
float32 fy
float32 cx
float32 cy
uint32 height
uint32 width


sensor_msgs/Image img		#keyframe image
sensor_msgs/Image idep		#keyframe inverse depth 
