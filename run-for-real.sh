if [ -z "$2" ]; then
    echo "Usage: $0 ROS_DOMAIN_ID FILE_TO_EXECUTE"
    echo "Example: $0 42 tb3.py"
    exit 1
fi
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=$1
python3 $2
