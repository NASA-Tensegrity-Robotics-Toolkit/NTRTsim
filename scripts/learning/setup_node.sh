apt-get -y install g++ libglib2.0-dev curl freeglut3 freeglut3-dev cmake build-essential unzip git rsync python-pip python-dev screen

git clone https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim.git

cd NTRTsim

./setup.sh
./setup.sh

cd bin

./build.sh

pip install psutil
