int8 TYPE_BOX=0
int8 TYPE_SPHERE=1
int8 TYPE_PLANE=2

#the name of the object
string name

#if you leave it empty, all collisions are checked 
string[] allowedCollisions

#the frame, in which the object is added
string parent_frame

#the transformation relative to the parent_frame
geometry_msgs/TransformStamped transformation

#depends on the type
float64[] size

#see constants
int8 type

#the color
std_msgs/ColorRGBA color