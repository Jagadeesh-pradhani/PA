ARG1=$1

if [ "$ARG1" == "circle" ]; then
   echo "circling";
   rosrun rss_linux_pkg move_turtlebot_circle.py 

elif [ "$ARG1" == 'forward_backward' ]; then
     echo "back and forth";
     rosrun rss_linux_pkg move_turtlebot_forward_backward.py

else 
echo "Please enter one of the following;
circle
forward_backward"

fi
