#include <iostream>
#include <ev3dev.h>

#include <sys/time.h>
#include <unistd.h>


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

const float PI = 3.14159265;

class DifferentialBot
{
public:
    DifferentialBot( ev3dev::motor& sourceMotorLeft, ev3dev::motor& sourceMotorRight ) :
        motorLeft(sourceMotorLeft),
        motorRight(sourceMotorRight),
        axleTrack(5040), // mils
        tireDiameter(3709) // mils
    {
        tireCircumference = long( PI * float(tireDiameter) );
    };

    long getLeftMotorDegrees()
    {
        return motorLeftDegrees;
    };

    long getRightMotorDegrees()
    {
        return motorRightDegrees;
    };

    long getAxleTrack()
    {
        return axleTrack;
    };

    long getTireDiameter()
    {
        return tireDiameter;
    };

    long getTireCircumference()
    {
        return tireCircumference;
    };

    float degreesPerInch()
    {
        const float milsPerInch = 1000.0;
        return 360.0 * milsPerInch / float(getTireCircumference());
    };

    void captureDegreesValues()
    {
        motorLeftDegrees    = motorLeft.position();
        motorRightDegrees   = motorRight.position();
    };

    ev3dev::motor& motorLeft;
    ev3dev::motor& motorRight;

private:

    long motorLeftDegrees;
    long motorRightDegrees;

    long axleTrack;
    long tireDiameter;
    long tireCircumference;
};

class DifferentialPidController
{
public:
    DifferentialPidController( DifferentialBot& aDiffBot ) :
        diffBot(aDiffBot),
        k_Pnumerator(7),
        k_Pdenomenator(5),
        k_Inumerator(1),
        k_Idenomenator(1),
        k_Dnumerator(1),
        k_Ddenomenator(4)
    {

    };

    void init( long speed, long rampTime )
    {
        intendedMotorSpeed = speed;
        maxSignal = speed * 1 / 5;

std::cout << "dpidCtl init" << std::endl;

        diffBot.motorLeft
            .set_duty_cycle_sp(speed)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake")
            .set_position(0);

        diffBot.motorRight
            .set_duty_cycle_sp(speed)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake")
            .set_position(0);

    };

    void start()
    {
        diffBot.motorLeft.run_direct();
        diffBot.motorRight.run_direct();
        eSum = 0;
        eOld = 0;
        tOld = usNow();
    };

    void doControlFeedbackFrame()
    {
        unsigned long tN = usNow();
        long dt = tN - tOld;

        long e = diffBot.getLeftMotorDegrees() - diffBot.getRightMotorDegrees();
        long D = ( e - eOld );

        eSum += e; // accumulate error over time

        long sP = k_Pnumerator * e / k_Pdenomenator;
        // long sI = k_Inumerator * eSum / k_Idenomenator;
        long sI = julery_isqrt( k_Inumerator * eSum / k_Idenomenator );
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

        diffBot.motorLeft.set_duty_cycle_sp( intendedMotorSpeed - signal );
        diffBot.motorRight.set_duty_cycle_sp( intendedMotorSpeed + signal );

        std::cout << tN << " dt=" << dt << ", e=" << e << ", sP=" << sP << ", eSum=" << eSum << ", sI=" << sI << ", D=" << D << ", sD=" << sD << ", signal=" << signal << std::endl;
        // if(sD){
        //     std::cout << "     " << " e=" << e << ", eOld=" << eOld << ", dt=" << dt << ", D=" << D << ", k_Dnumerator=" << k_Dnumerator << ", k_Ddenomenator=" << k_Ddenomenator << ", sD=" << sD << std::endl;
        //     // break;
        // }

        eOld = e;   // retain current error value
        tOld = tN;  // retain current time value
    };

    void stop()
    {
        diffBot.motorLeft.stop();
        diffBot.motorRight.stop();
    };

private:
    const long k_Pnumerator;
    const long k_Pdenomenator;
    const long k_Inumerator;
    const long k_Idenomenator;
    const long k_Dnumerator;
    const long k_Ddenomenator;

    DifferentialBot& diffBot;

    long maxSignal;
    long intendedMotorSpeed;

    long eSum;
    long eOld;
    long tOld;
};

class Odometer
{
public:
    Odometer( DifferentialBot& aDiffBot ) :
       x(0.0),
       y(0.0),
       phi(0.0),
       diffBot(aDiffBot),
       axleTrack(float(aDiffBot.getAxleTrack())),
       tireDiameter(float(aDiffBot.getTireDiameter()))
    {
        float R = tireDiameter / 2.0;
        dw2ds = R * PI / 180.0;

        std::cout << "odometer begin "
                  << "  dw2ds=" << dw2ds
                  << std::endl;
    };

    void reset()
    {
        oldLeftDegrees = diffBot.getLeftMotorDegrees();
        oldRightDegrees = diffBot.getRightMotorDegrees();
        std::cout << "odometer "
                  << "  oldLeftDegrees=" << oldLeftDegrees
                  << "  oldRightDegrees=" << oldRightDegrees
                  << std::endl;

        for( int x=0; x<80; x++ ) {
            for( int y=0; y<80; y++ ) {
                plotData[x][y] = '.';
            }
        }


    };

    #define min(a,b) ( (a)<(b) ? (a) : (b) )
    #define max(a,b) ( (a)>(b) ? (a) : (b) )

    void update()
    {
        // do something here
        long  wr  = diffBot.getRightMotorDegrees();
        long  wl  = diffBot.getLeftMotorDegrees();
        float dwr = float( wr - oldRightDegrees );
        float dwl = float( wl - oldLeftDegrees );
        oldRightDegrees = wr;
        oldLeftDegrees  = wl;
        float dsl = dwl * dw2ds;
        float dsr = dwr * dw2ds;
        float ds  = ( dsr + dsl ) / 2.0;

        x += ds * cos( phi );
        y += ds * sin( phi );
        phi += ( dsr - dsl) / axleTrack;

        int plotX = int(x/100);
        int plotY = int(y/100);
        plotX = max( 0, min( 79, plotX ) );
        plotY = max( 0, min( 79, plotY ) );

        std::cout << "  x=" << x
                  << "  y=" << y
                  << "  phi=" << phi
                  // << "  wl=" << wl
                  // << "  wr=" << wr
                  // << "  dsl=" << dsl
                  // << "  dsr=" << dsr
                  << std::endl;

        plotData[ plotX ][ plotY ] = '*';
    };

    void printPlot()
    {
        for( int y=79; y>=0; y-- ) {
            for( int x=0; x<80; x++ ) {
                std::cout << plotData[x][y] << ' ';
            }
            std::cout << std::endl;
        }
    }

private:

    char plotData[80][80];

    float             x, y, phi;
    float             axleTrack, tireDiameter;
    float             dw2ds;
    long              oldLeftDegrees, oldRightDegrees;
    DifferentialBot&  diffBot;
};

void turnWheelOnce( ev3dev::motor& theMotor, DifferentialBot& diffBot, Odometer& odometer )
{
    theMotor
        // .set_duty_cycle_sp(25)
        .set_duty_cycle_sp(25)
        .set_ramp_up_sp(250);

    long startPosition = theMotor.position();

std::cout << "turnWheelOnce ::   startPosition: " << startPosition << std::endl;

    usleep( 250 * 1000 );

std::cout << "turnWheelOnce ::   theMotor.position: " << theMotor.position() << std::endl;

    theMotor.run_direct();

    while( theMotor.position() < 150+startPosition )
    {
        usleep( 16 * 1000 );
        diffBot.captureDegreesValues();
        odometer.update();
    }

    theMotor
        .set_duty_cycle_sp(13);

    while( theMotor.position() < 180+startPosition )
    {
        usleep( 16 * 1000 );
        diffBot.captureDegreesValues();
        odometer.update();
    }

    theMotor.stop();
}

int main( int argc, char* argv[] )
{
    ev3dev::motor   leftMotor(ev3dev::OUTPUT_B);
    ev3dev::motor   rightMotor(ev3dev::OUTPUT_C);
    DifferentialBot diffBot( leftMotor, rightMotor );

    long degrees = 0;
    if( argc == 2 )
    {
        float inches = atof(argv[1]);
        degrees = long( inches * diffBot.degreesPerInch() );
        std::cout << "inches: " << inches << "   diffBot.degreesPerInch(): " << diffBot.degreesPerInch() << "   degrees: " << degrees << std::endl;
    }
    else
    {
        std::cout << "Usage: " << argv[0] << " {D}" << std::endl;
        std::cout << "   D    the distance to travel (inches)" << std::endl;
        exit(0);
    }

    Odometer odometer( diffBot );
    DifferentialPidController pidControl( diffBot );
    pidControl.init( 25, 250 ); // max-speed, ramp-time-ms
    diffBot.captureDegreesValues();
    odometer.reset();

    usleep( 250 * 1000 );

    pidControl.start();

    unsigned long tStart = usNow();
    long loopCount = 0;

    do
    {
        usleep( 16 * 1000 );
        ++loopCount;
        diffBot.captureDegreesValues();
        odometer.update();
        pidControl.doControlFeedbackFrame();
    }
    while( diffBot.getLeftMotorDegrees() < degrees );

    pidControl.stop();


    usleep( 250 * 1000 );
    turnWheelOnce( rightMotor, diffBot, odometer );
    turnWheelOnce( rightMotor, diffBot, odometer );

    usleep( 250 * 1000 );
    turnWheelOnce( leftMotor, diffBot, odometer );

    std::cout << "   motorLeft.position() :  " << diffBot.getLeftMotorDegrees()  << std::endl;
    std::cout << "   motorRight.position() : " << diffBot.getRightMotorDegrees() << std::endl;

    unsigned long usTotal = usNow() - tStart;
    unsigned long dtAverage = usTotal / loopCount;

    std::cout << "   usTotal=" << usTotal << "   loopCount=" << loopCount << "   dtAverage=" << dtAverage << "us" << std::endl;

    odometer.printPlot();

}

