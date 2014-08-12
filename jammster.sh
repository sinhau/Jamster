echo " "
echo "Peripherals Host"
echo "----------------"
rosrun baxter_bridge peripherals_host_mod.py  &
sleep 1
echo " "
echo "Joint Controller Host"
echo "---------------------"
rosrun baxter_bridge jointcontroller_host_mod.py &
sleep 1
echo " "
echo "Left Hand Camera"
echo "----------------"
rosrun baxter_bridge compressed_camera_host.py left_hand_camera &
sleep 1
echo " "
echo "Right Hand Camera"
echo "-----------------"
rosrun baxter_bridge compressed_camera_host.py right_hand_camera  &
sleep 1
echo " "
echo "Head Camera"
echo "-----------"
rosrun baxter_bridge compressed_camera_host.py head_camera & 
sleep 1
#echo "Wheelchair server"
#echo "-----------"
#sudo python '/home/cats/Jamster/wheelchairRRService.py' &
sleep 10
#echo " "
#echo "Matlab"
#echo "-----------"
#matlab -nodesktop -r "mainJamboxx" 
