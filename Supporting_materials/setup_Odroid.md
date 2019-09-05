# Setting up the Odroid XU4

## Initial setup

Download Ubuntu 18.04 image for Odroid from here:  
https://wiki.odroid.com/odroid-xu4/os_images/linux/ubuntu_4.14/ubuntu_4.14  
And put on SD or eMMC card for Odroid  


Power on Odroid  
It will shut down automatically if this is the first time it's been turned on  
Turn it on again  

Now when it restarts, update linux  
$ sudo apt update  
$ sudo apt upgrade  

I received an error:  
"depmod ... module not found"  
-> according to forum this is not a problem https://forum.odroid.com/viewtopic.php?t=33481

$ sudo apt dist-upgrade
$ sudo apt autoremove
$ sudo apt install linux-image-xu3

I received an error:  
"unable to locate package"  
-> apparently not a problem, the previously dist-upgrade step should have been sufficient https://forum.odroid.com/viewtopic.php?t=31484

## Tools and dependencies
Create workspace directory  
$ mkdir ws  

### Git
$ sudo apt install git

### Eigen
$ sudo apt-get install libeigen3-dev

Note: I previously installed Eigen using the Github hosted source, but later had problems with g2o finding it

### OpenCV
$ cd ~/ws
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev  
$ wget https://github.com/opencv/opencv/archive/3.4.2.zip  
$ unzip 3.4.2.zip  
$ cd ~/opencv  
$ mkdir build  
$ cd build  
$ cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..  
$ make -j4  
$ sudo make install  

Note: I was not able to get OpenCV to install with Eigin (couldn't get it to find Eigen properly), though this is not actually required

### Pangolin
$ cd ~/ws
$ git clone https://github.com/stevenlovegrove/Pangolin.git  
$ sudo apt-get install libglew-dev  
$ cd Pangolin  
$ mkdir build  
$ cd build  
$ cmake ..  
$ cmake --build .  


### ORB_SLAM2_PRJ
$ git clone https://github.com/matt769/ORB_SLAM2_PRJ  
$ cd ORB_SLAM2_PRJ  
$ ./build.sh  

Note: The build.sh file is modified to include the -j2 argument for make when building ORB SLAM. Without this the ORB SLAM build will probably fail (on the Odroid) due to memory issues (though g2o and DBoW2 should be ok). For systems with more memory, this can be changed back to -j.


### FlyCapture
See this page:  
https://www.flir.co.uk/support-center/iis/machine-vision/application-note/getting-started-with-flycapture-2-and-arm/

I have put the file on dropbox
$ cd ~/ws
$ wget https://www.dropbox.com/s/503t7ibigvr35u0/flycapture.2.13.3.31_armhf_xenial.tar.gz?dl=0
$ mv flycapture.2.13.3.31_armhf_xenial.tar.gz?dl=0 flycapture.2.13.3.31_armhf_xenial.tar.gz
$ tar -xzf flycapture.2.13.3.31_armhf_xenial.tar.gz

$ cd flycapture.2.13.3.31_armhf/lib  
$ sudo cp libflycapture* /usr/lib  
$ sudo sh flycap2-conf  
I used user name "odroid"  
$ cd src/FlyCapture2Test  
$ make  




## Networking
Ubuntu 18.04 uses a new system (not interfaces file anymore)  

Place fillowing file 'config.yaml' (actual name doesn't matter) in /etc/netplan/  
</pre>
network:  
  version: 2  
  renderer: networkd  
  ethernets:  
    enp3s0:  
      addresses:  
        - 169.254.130.11/24  
</pre>


## Serial port permissions
This gives read/write permissions on ttyACM0  

$ sudo adduser odroid dialout  

Actually the odroid user is already a member of the dialout group. This is not the case with a standard Ubuntu install (e.g. on a laptop) I think.



## Teensy rules

Download the device rules from here https://www.pjrc.com/teensy/td_download.html
$ sudo cp 49-teensy.rules /etc/udev/rules.d/



## For normal computer
Process will be basically the same, but ensure that the correct version of the FlyCapture software is installed e.g. AMD64 for my laptop.