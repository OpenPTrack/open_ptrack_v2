ROS_MASTER_URI=http://$1:11311/
ROS_IP=$2
ROS_PC_NAME="$3"

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "ROS_PC_NAME=$ROS_PC_NAME"
echo "actual ip addresses: $(hostname -I)"
haveRosIp=false
for word in $(hostname -I)
do
        if [ "$ROS_IP" == "$word" ] 
        then
                haveRosIp=true
        fi
done

if [ "$haveRosIp" = false ] ; then
        RED='\033[0;31m'
        NC='\033[0m' # No Color
        echo -e "${RED}ROS_IP is different from the actual address${NC}"
fi
