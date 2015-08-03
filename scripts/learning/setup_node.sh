apt-get update
apt-get -y install g++ libglib2.0-dev curl freeglut3 freeglut3-dev cmake build-essential unzip git rsync python-pip python-dev screen

git clone https://github.com/NASA-Tensegrity-Robotics-Toolkit/NTRTsim.git

pushd NTRTsim/

./setup.sh
./setup.sh

popd > /dev/null

pushd NTRTsim/bin/
cd bin
./build.sh
popd > /dev/null

pip install psutil

