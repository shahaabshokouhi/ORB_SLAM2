if [ -z "$REALSENSE_CONFIG" ]; then
    echo "Error: REALSENSE_CONFIG environment variable is not set."
    echo "Set it in ~/.bashrc, e.g.: export REALSENSE_CONFIG=/path/to/realsense.yaml"
    exit 1
fi
./Examples/rgbd_live Vocabulary/ORBvoc.txt "$REALSENSE_CONFIG"
