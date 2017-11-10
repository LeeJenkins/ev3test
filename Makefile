
RPATH_SYSROOT=-Wl,-rpath=/var/armv5/sysroot/lib:/var/armv5/sysroot/usr/lib
DYNAMIC_SYSROOT=-Wl,--dynamic-linker=/var/armv5/sysroot/lib/ld-linux.so.3
EV3CPPDIR=/opt/ev3cpp
CXX=arm-none-linux-gnueabi-g++
CCOPTS=-std=c++11

ROBOT_IP_ADDR=ev3

PROGRAM=behaviorBot
# PROGRAM=seg337

BOTFILE: ${PROGRAM}
	echo "use ssh connnection sharing to speed this up"
	sshpass -f ./robot-password.ev3 scp ${PROGRAM} robot@${ROBOT_IP_ADDR}:~
	sshpass -f ./robot-password.ev3 ssh robot@${ROBOT_IP_ADDR} aplay R2D2e.wav

${PROGRAM}: ${PROGRAM}.cpp
	${CXX} ${PROGRAM}.cpp ${CCOPTS} -o ${PROGRAM} -I ${EV3CPPDIR} -L ${EV3CPPDIR} -lev3dev ${RPATH_SYSROOT} ${DYNAMIC_SYSROOT}
