export ROS_LOCALHOST_ONLY=1
if [ -z "$1" ]; then
    echo "Usage: $0 FILE_TO_EXECUTE"
    echo "Example: $0 tb3.py"
fi
python3 $1
