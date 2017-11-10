#include <iostream>
#include <ev3dev.h>

#include <sys/time.h>
#include <unistd.h>

void runMotorSpeedTime( ev3dev::motor& motor, int speed_sp, int time_sp )
{
    if( motor.connected() )
    {
        motor
            .set_speed_sp(speed_sp)
            .set_time_sp(time_sp)
            .run_timed();
    }
}

void runConstant( ev3dev::motor& motorLeft, ev3dev::motor& motorRight )
{
    int speed_tps = 500;
    int time_ms = 3000;

    // initialize both motor rotation encoders to zero
    motorLeft.set_position(0);
    motorRight.set_position(0);

    runMotorSpeedTime( motorLeft, speed_tps, time_ms );
    runMotorSpeedTime( motorRight, speed_tps, time_ms );

    usleep( time_ms * 1000 + 250 );

    std::cout << "   motorLeft.position() :  " << motorLeft.position()  << std::endl;
    std::cout << "   motorRight.position() : " << motorRight.position() << std::endl;

}

unsigned long usNow()
{
    struct timeval timeValue;
    gettimeofday( &timeValue, NULL );
    return ( 1000 * 1000 * timeValue.tv_sec ) + timeValue.tv_usec;
}

/* by Jim Ulery, modified to allow for negative numbers */
static long julery_isqrt(long val) {
    long sign = 1;
    if( val < 0 ) {
        val = - val;
        sign = -1;
    }
    long temp, g=0, b = 0x8000, bshft = 15;
    do {
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g * sign;
}

void driveStraightDT( ev3dev::motor& motorLeft, ev3dev::motor& motorRight )
{
    const long intendedMotorSpeed = 50;
    const long maxSignal = intendedMotorSpeed * 1 / 5;
    const long k_Pnumerator = 4;
    const long k_Pdenomenator = 1;
    const long k_Inumerator = 1;
    const long k_Idenomenator = 14 * 1000;
    const long k_Dnumerator = 3 * 1000;
    const long k_Ddenomenator = 1;

    // initialize both motor rotation encoders to zero
    motorLeft.set_position(0);
    motorRight.set_position(0);

    // start the motors
    motorLeft
        .set_duty_cycle_sp(intendedMotorSpeed)
        .run_direct();
    motorRight
        .set_duty_cycle_sp(intendedMotorSpeed)
        .run_direct();

    std::cout << usNow() << " trying to drive straight" << std::endl;

    unsigned long usRunTime = 5 * 1000 * 1000;
    unsigned long tN = usNow();
    unsigned long tOld = tN;
    unsigned long usEndTime = tN + usRunTime;

    long I = 0;
    long eOld = 0.0;
    long loopCount = 0;

    do
    {
        usleep(10);
        tN = usNow();
        long dt = tN - tOld;
        if( !dt ) continue; // this could happen, e.g. if clock is only accurate to 100us

        ++loopCount;

        long e = motorLeft.position() - motorRight.position();
        long D = ( e - eOld ) / dt;

        I += e * dt; // accumulate error over time

        // long signal =  k_Pnumerator * e / k_Pdenomenator +
        //                k_Inumerator * I / k_Idenomenator +
        //                k_Dnumerator * D / k_Ddenomenator;

        long sP = k_Pnumerator * e / k_Pdenomenator;
        long sI = julery_isqrt( k_Inumerator * I / k_Idenomenator );
        // long sD = k_Dnumerator * D / k_Ddenomenator;
        long sD = k_Dnumerator * ( e - eOld ) / ( dt * k_Ddenomenator );

        long signal = sP + sI + sD;

        if( signal > maxSignal )
        {
            signal = maxSignal;
        }
        else if( signal < -maxSignal )
        {
            signal = -maxSignal;
        }

        motorLeft.set_duty_cycle_sp( intendedMotorSpeed - signal );
        motorRight.set_duty_cycle_sp( intendedMotorSpeed + signal );

// std::cout << tN << " dt=" << dt << ", e=" << e << ", sP=" << sP << ", I=" << I << ", sI=" << sI << ", D=" << D << ", sD=" << sD << ", signal=" << signal << std::endl;
// if(sD){
//     std::cout << "     " << " e=" << e << ", eOld=" << eOld << ", dt=" << dt << ", D=" << D << ", k_Dnumerator=" << k_Dnumerator << ", k_Ddenomenator=" << k_Ddenomenator << ", sD=" << sD << std::endl;
//     // break;
// }


        eOld = e;   // retain current error value
        tOld = tN;  // retain current time value
    }
    // continue while "now" is less than the end time
    while( tN < usEndTime );

    motorLeft.stop();
    motorRight.stop();

    std::cout << usNow() << " finished driving straight" << std::endl;
    std::cout << "  " << usRunTime << std::endl;

    std::cout << "   motorLeft.position() :  " << motorLeft.position()  << std::endl;
    std::cout << "   motorRight.position() : " << motorRight.position() << std::endl;

    unsigned long usTotal = usRunTime + tN - usEndTime;
    unsigned long dtAverage = usTotal / loopCount;

    std::cout << "   usTotal=" << usTotal << "   loopCount=" << loopCount << "   dtAverage=" << dtAverage << std::endl;

}

void driveStraight( ev3dev::motor& motorLeft, ev3dev::motor& motorRight )
{
    const long intendedMotorSpeed = 20;
    const long maxSignal = intendedMotorSpeed * 1 / 5;
    const long k_Pnumerator = 7;
    const long k_Pdenomenator = 5;
    const long k_Inumerator = 1;
    const long k_Idenomenator = 1;
    const long k_Dnumerator = 1;
    const long k_Ddenomenator = 4;

    // initialize both motor rotation encoders to zero
    motorLeft.set_position(0);
    motorRight.set_position(0);

    // start the motors
    motorLeft
        .set_duty_cycle_sp(intendedMotorSpeed)
        .set_ramp_up_sp(250)
        .run_direct();
    motorRight
        .set_duty_cycle_sp(intendedMotorSpeed)
        .set_ramp_up_sp(250)
        .run_direct();

    std::cout << usNow() << " trying to drive straight" << std::endl;

    unsigned long usRunTime = 3 * 1000 * 1000;
    unsigned long tN = usNow();
    unsigned long tOld = tN;
    unsigned long usEndTime = tN + usRunTime;

    long I = 0;
    long eOld = 0.0;
    long loopCount = 0;

    do
    {
        usleep( 16 * 1000 );
        tN = usNow();
        long dt = tN - tOld;
        // if( !dt ) continue; // this could happen, e.g. if clock is only accurate to 100us

        ++loopCount;

        long e = motorLeft.position() - motorRight.position();
        // long e = ( ( 205 * motorLeft.position() ) / 200 ) - motorRight.position();
        long D = ( e - eOld );

        I += e; // accumulate error over time

        // long signal =  k_Pnumerator * e / k_Pdenomenator +
        //                k_Inumerator * I / k_Idenomenator +
        //                k_Dnumerator * D / k_Ddenomenator;

        long sP = k_Pnumerator * e / k_Pdenomenator;
        long sI = julery_isqrt( k_Inumerator * I / k_Idenomenator );
        // long sI = ( k_Inumerator * I / k_Idenomenator );
        // long sD = k_Dnumerator * D / k_Ddenomenator;
        long sD = k_Dnumerator * ( D * D * D ) / k_Ddenomenator;

        long signal = sP + sI + sD;

        if( signal > maxSignal )
        {
            signal = maxSignal;
        }
        else if( signal < -maxSignal )
        {
            signal = -maxSignal;
        }

        motorLeft.set_duty_cycle_sp( intendedMotorSpeed - signal );
        motorRight.set_duty_cycle_sp( intendedMotorSpeed + signal );

        std::cout << tN << " dt=" << dt << ", e=" << e << ", sP=" << sP << ", I=" << I << ", sI=" << sI << ", D=" << D << ", sD=" << sD << ", signal=" << signal << std::endl;
        // if(sD){
        //     std::cout << "     " << " e=" << e << ", eOld=" << eOld << ", dt=" << dt << ", D=" << D << ", k_Dnumerator=" << k_Dnumerator << ", k_Ddenomenator=" << k_Ddenomenator << ", sD=" << sD << std::endl;
        //     // break;
        // }

        eOld = e;   // retain current error value
        tOld = tN;  // retain current time value
    }
    // continue while "now" is less than the end time
    while( tN < usEndTime );

    motorLeft.stop();
    motorRight.stop();

    std::cout << usNow() << " finished driving straight" << std::endl;
    std::cout << "  " << usRunTime << std::endl;

    std::cout << "   motorLeft.position() :  " << motorLeft.position()  << std::endl;
    std::cout << "   motorRight.position() : " << motorRight.position() << std::endl;

    unsigned long usTotal = usRunTime + tN - usEndTime;
    unsigned long dtAverage = usTotal / loopCount;

    std::cout << "   usTotal=" << usTotal << "   loopCount=" << loopCount << "   dtAverage=" << dtAverage << std::endl;
    std::cout << "   error sum = " << I << std::endl;

}

int main( int argc, char* argv[] )
{
    std::cout << "hello, ev3 world! this is ev3test." << std::endl;

    ev3dev::motor motorLeft(ev3dev::OUTPUT_B);
    ev3dev::motor motorRight(ev3dev::OUTPUT_C);

    usleep( 250 * 1000 );

    driveStraight(motorLeft,motorRight);
    // runConstant( motorLeft, motorRight );

    std::cout << "goodbye, ev3 world." << std::endl;
}

