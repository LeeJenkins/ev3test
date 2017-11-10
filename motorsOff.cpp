#include <iostream>
#include <ev3dev.h>

int main( int argc, char* argv[] )
{
    ev3dev::motor motorLeft(ev3dev::OUTPUT_B);
    ev3dev::motor motorRight(ev3dev::OUTPUT_C);

    motorLeft.stop();
    motorRight.stop();
}

