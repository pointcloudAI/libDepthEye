
sudo cp ./config/ubuntu/99-DepthEye.rules /etc/udev/rules.d/

sudo chmod a+x /etc/udev/rules.d/99-DepthEye.rules
sudo udevadm control --reload
