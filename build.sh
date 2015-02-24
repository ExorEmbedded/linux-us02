make ARCH=arm socfpga_defconfig
make ARCH=arm CROSS_COMPILE=/opt/altera-linux/linaro/gcc-linaro-arm-linux-gnueabihf-4.7-2012.11-20121123_linux/bin/arm-linux-gnueabihf- -j12 zImage dtbs

