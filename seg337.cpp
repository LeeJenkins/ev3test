
#include <algorithm>
#include <vector>
#include <iostream>
#include <math.h>
#include <ev3dev.h>

#include <sys/time.h>
#include <unistd.h>


unsigned long usNow()
{
    struct timeval timeValue;
    gettimeofday( &timeValue, NULL );
    return ( 1000 * 1000 * timeValue.tv_sec ) + timeValue.tv_usec;
}

/* by Jim Ulery */
long julery_isqrt(long val) {
    long temp, g=0, b = 0x8000, bshft = 15;
    do {
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g;
}

long ortho_isqrt(long val) {
    long sign = 1;
    if( val < 0 ) {
        val = - val;
        sign = -1;
    }
    return julery_isqrt(val) * sign;
}

// returns the square of a number, but preserving
// the sign, so that e.g. orthoSquare(-2) returns -4
long ortho_square( long value )
{
    long sign = 1;
    if( value < 0 ) {
        sign = -1;
    }
    return sign * value * value;
}

long idist( long x0, long y0, long x1, long y1 )
{
    long dx = x1 - x0;
    long dy = y1 - y0;
    return julery_isqrt( dx*dx + dy*dy );
}

float distance( float x0, float y0, float x1, float y1 )
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    return sqrt( dx*dx + dy*dy );
}

const float PI = 3.14159265;
const float rad2Degrees = 180.0 / PI;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class SegBot (MVC:MODEL)
// * brief models the physical attributes of the robot
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class SegBot
{
public:
    SegBot() :
        motorLeft(ev3dev::OUTPUT_D),
        motorRight(ev3dev::OUTPUT_A),
        axleTrack(5040), // mils
        tireDiameter(3709) // mils
    {
        tireCircumference = long( PI * float(tireDiameter) );
    };

    // this method should only ever be called by the BotManager
    void captureFrame()
    {
        const float rightFactor = 2874.0 / 2891.0 ;
        motorLeftDegrees    = motorLeft.position();
        motorRightDegrees   = long( float(motorRight.position()) * rightFactor );
    };

    long getLeftMotorDegrees()
    {
        return motorLeftDegrees;
    };

    long getRightMotorDegrees()
    {
        return motorRightDegrees;
    };

    ev3dev::motor& getLeftMotor()
    {
        return motorLeft;
    };

    ev3dev::motor& getRightMotor()
    {
        return motorRight;
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
        // // compensate for independently measured distance travelled
        // const float errorFactor = 39.0 / 38.0;
        // return 360.0 * milsPerInch * errorFactor / float(getTireCircumference());
    };

private:

    ev3dev::motor motorLeft;
    ev3dev::motor motorRight;

    long motorLeftDegrees;
    long motorRightDegrees;

    long axleTrack;
    long tireDiameter;
    long tireCircumference;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class Behavior (MVC:MODEL)
// * brief keeps track of the robot;s location and orientation
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class Odometer
{
public:
    Odometer( SegBot& asegBot ) :
       x(0.0),
       y(0.0),
       phi(0.0),
       segBot(asegBot),
       axleTrack(float(asegBot.getAxleTrack())),
       tireDiameter(float(asegBot.getTireDiameter()))
    {
        float R = tireDiameter / 2.0;
        dw2ds = R * PI / 180.0;

        std::cout << "odometer begin "
                  << "  dw2ds=" << dw2ds
                  << std::endl;
    };

    void reset()
    {
        segBot.getLeftMotor().set_position(0);
        segBot.getRightMotor().set_position(0);

        oldLeftDegrees = segBot.getLeftMotorDegrees();
        oldRightDegrees = segBot.getRightMotorDegrees();

        // std::cout << "odometer "
        //           << "  oldLeftDegrees=" << oldLeftDegrees
        //           << "  oldRightDegrees=" << oldRightDegrees
        //           << std::endl;

        for( int x=0; x<80; x++ ) {
            for( int y=0; y<80; y++ ) {
                plotData[x][y] = '.';
            }
        }


    };

    void update()
    {
        // do something here
        long  wr  = segBot.getRightMotorDegrees();
        long  wl  = segBot.getLeftMotorDegrees();
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

        // std::cout << "  x=" << x
        //           << "  y=" << y
        //           << "  phi=" << phi
        //           // << "  wl=" << wl
        //           // << "  wr=" << wr
        //           // << "  dsl=" << dsl
        //           // << "  dsr=" << dsr
        //           << std::endl;

        // int plotX = int(x/100);
        // int plotY = int(y/100);
        // plotX = std::max( 0, std::min( 79, plotX ) );
        // plotY = std::max( 0, std::min( 79, plotY ) );
        // plotData[ plotX ][ plotY ] = '*';
    };

    void printPlot()
    {
        for( int y=79; y>=0; y-- ) {
            for( int x=0; x<80; x++ ) {
                std::cout << plotData[x][y] << ' ';
            }
            std::cout << std::endl;
        }
    };

    float X()   { return x; };
    float Y()   { return y; };
    float PhiRadians() { return phi; };
    float PhiDegrees() { return phi * rad2Degrees; };

private:

    char plotData[80][80];

    float             x, y, phi;
    float             axleTrack, tireDiameter;
    float             dw2ds;
    long              oldLeftDegrees, oldRightDegrees;
    SegBot&  segBot;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class Behavior (MVC:CONTROLLER)
// * brief generic interface for various robot behaviors
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class Behavior
{
public:
    Behavior()
    {
    };
    virtual ~Behavior()
    {
    };
    virtual void start() = 0;
    virtual void update() = 0;
    virtual bool isDone() = 0;
    virtual void stop() = 0;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorStandStraight (MVC:CONTROLLER)
// * brief stand up straight without falling over
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorStandStraight : public Behavior
{
public:
    BehaviorStandStraight( 
        ev3dev::motor& footMotor,
        ev3dev::gyro_sensor& pitchGyro
    ):
        myMotor(footMotor),
        gyro(pitchGyro),
        k_P(25.0),
        k_I(0.0),
        k_D(5)
    {
        maxSignal = 100;
    };

    ~BehaviorStandStraight() { };

    void start()
    {
        myMotor
            .set_duty_cycle_sp(0)
            .set_ramp_up_sp(0)
            .run_direct();

        eSum = 0;
        eOld = 0;
    };

    void update()
    {
        static int ucount = 0;

        float e = gyro.value();
        float D = ( e - eOld );

        eSum += e; // accumulate error over time

        float u = k_P*e + k_I*eSum + k_D*D;

        if( u > maxSignal )
        {
            u = maxSignal;
        }
        else if( u < -maxSignal )
        {
            u = -maxSignal;
        }

        myMotor.set_duty_cycle_sp( - u );

if( ++ucount == 100 )
{
std::cout << "e=" << e << ", u=" << u << std::endl;
ucount = 0;
}

        eOld = e;   // retain current error value
    };

    bool isDone()    {
        return false;
    };

    void stop()
    {
        myMotor.stop();
    };

private:
    const float k_P;
    const float k_I;
    const float k_D;

    ev3dev::motor& myMotor;
    ev3dev::gyro_sensor& gyro;

    float eSum;
    float eOld;
    float maxSignal;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BotManager (MVC:CONTROLLER)
// * brief manages frames and selects behaviors
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BotManager
{
public:
    BotManager()
    {
    };

    void run()
    {
std::cout << "configure the segbot" << std::endl;

        SegBot segBot;

        Odometer odometer(segBot);
        odometer.reset();

        // ev3dev::sound::speak("Hello, I am Robot!", true);

        ev3dev::gyro_sensor gyro(ev3dev::INPUT_1);
std::cout << "ev3dev::gyro_sensor on " << ev3dev::INPUT_1 << std::endl;

std::cout << "configure the gyro sensor for CALIBRATION" << std::endl;
        // gyro.rate();
        gyro.set_mode(gyro.mode_gyro_cal);
        usleep( 1000 * 1000 );

std::cout << "configure the gyro sensor for ANGLE" << std::endl;
        // gyro.angle();
        gyro.set_mode(gyro.mode_gyro_ang);
        usleep( 1000 * 1000 );


        BehaviorStandStraight behaviorStandStraightLeft( segBot.getLeftMotor(), gyro );
        BehaviorStandStraight behaviorStandStraightRight( segBot.getRightMotor(), gyro );

        unsigned long tStart = usNow();
        long frameCount = 0;

            behaviorStandStraightLeft.start();
            behaviorStandStraightRight.start();
            do
            {
                ++frameCount;

                segBot.captureFrame();
                odometer.update();
                behaviorStandStraightLeft.update();
                behaviorStandStraightRight.update();
                // usleep( 10 * 1000 );
            }
            while( !ev3dev::button::back.pressed() );

            behaviorStandStraightLeft.stop();
            behaviorStandStraightRight.stop();

    };
private:
};


int main( int argc, char* argv[] )
{
    BotManager botManager;
    botManager.run();
}
