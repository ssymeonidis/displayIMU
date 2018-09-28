all:
	$(MAKE) -C imu all
	$(MAKE) -C display all

clean:
	rm bin/libIMU.so bin/displayIMU
