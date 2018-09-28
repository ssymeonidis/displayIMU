g++ -Wall -fPIC -o obj/IMU.o -c IMU.cpp
g++ -Wall -fPIC -o obj/fileUtils.o -c fileUtils.cpp
g++ -shared -o ../build/libIMU.so obj/IMU.o obj/fileUtils.o
