rm -f obj/IMU_quat.o IMU_quat.c IMU_quat.so
sed -e 's/inline //;s/static //' ../../imu/IMU_quat.h > IMU_quat.c
gcc -fPIC -c IMU_quat.c -o obj/IMU_quat.o
gcc -shared obj/IMU_quat.o -o IMU_quat.so
