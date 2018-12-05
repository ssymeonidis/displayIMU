rm -f temp/IMU_quat.org temp/IMU_quat.o
cp ../../imu/IMU_quat.h temp/IMU_quat.org
sed -e 's/inline //;s/static //' temp/IMU_quat.org > IMU_quat.c
gcc -fPIC -c IMU_quat.c -o temp/IMU_quat.o
gcc -shared temp/IMU_quat.o -o IMU_quat.so
