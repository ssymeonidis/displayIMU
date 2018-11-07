all:
	$(MAKE) -C imu all
	$(MAKE) -C display all
	$(MAKE) -C csv all

unit_test:
	$(MAKE) -C test clean
	$(MAKE) -C test all
	$(MAKE) -C test run

clean:
	rm -rf bin/displayIMU
	$(MAKE) -C imu clean
	$(MAKE) -C display clean
	$(MAKE) -C csv clean
