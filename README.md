# CAN-Node-Code Troubleshooting

If ./main won't run due to missing library file ending with '.so' (I think?):

	export LD_LIBRARY_PATH="/home/k3v/GitHubRepos/CAN-Node-Code/SupervisoryController/mysql-connector-c++-1.1.9-linux-ubuntu16.04-x86-64bit/lib"

		--> Will have to change start of library path to the correct directory of my-sql-connector-c++...



If PCAN won't connect, try

	sudo modprobe pcan


If not found, go to https://www.peak-system.com/forum/viewtopic.php?f=59&t=256 and follow the steps.  Here they are, in case the site goes down:

 	cd peak-linux-driver-7.4
 	make clean
 	make NET=NO
 	sudo make install
 	sudo modprobe pcan
	
check with 

	cat /proc/pcan that the driver was successfully installed.


If you can't find peak-linux-driver-7.4 folder, you may need to download it.  If, so follow these steps first then continue with those outlined above:

	1. 	Download the driver peak-linux-driver-7.4.tar.gz from our linux website and copy it into your home directory.

	2. 	Install libpopt-dev: sudo apt-get install libpopt-dev

	3. 	Install g++: 

			sudo apt-get install g++  

		(only necessary to build the test tool transmittest)

	
	4. 	unpack the driver: tar -xzf peak-linux-driver-7.4.tar.gz
