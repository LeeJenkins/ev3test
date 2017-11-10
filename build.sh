RPATH_SYSROOT=-Wl,-rpath=/var/armv5/sysroot/lib:/var/armv5/sysroot/usr/lib
DYNAMIC_SYSROOT=-Wl,--dynamic-linker=/var/armv5/sysroot/lib/ld-linux.so.3
EV3CPPDIR=/opt/ev3cpp
CXX=arm-none-linux-gnueabi-g++
CCOPTS=-std=c++11

target=$1

$CXX $target.cpp $CCOPTS -o $target -I $EV3CPPDIR -L $EV3CPPDIR -lev3dev $RPATH_SYSROOT $DYNAMIC_SYSROOT


