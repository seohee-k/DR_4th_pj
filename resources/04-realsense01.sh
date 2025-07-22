sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
sudo apt-get install apt-transport-https

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update -y

sudo apt-get remove --purge libgtk-3-dev -y

sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y

sudo apt-get install librealsense2-dev -y
sudo apt-get install librealsense2-dbg -y

realsense-viewer
