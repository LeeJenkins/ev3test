#include <algorithm>
#include <vector>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <ev3dev.h>

#include <sys/time.h>
#include <unistd.h>
#include <system_error>

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

inline float cm2mils( float cm )
{
    return cm / 0.00254;
}

const float PI = 3.14159265;

inline float rad2Degrees( float radians )
{
    return radians * 180.0 / PI;
}

inline float degrees2Rad( float degrees )
{
    return degrees * PI / 180.0;
}


class MedianFilter
{
public:
    MedianFilter()
    {
        newest = -9e99;
        recent = -9e99;
        oldest = -9e99;
        medianValue  = -9e99;
    };
    void init( float value )
    {
        newest = value;
        recent = value;
    };
    void push( float newValue )
    {
        oldest = recent;
        recent = newest;
        newest = newValue;
        if ((oldest <= recent) && (oldest <= newest))
        {
            medianValue = (recent <= newest) ? recent : newest;
        }
        else if ((recent <= oldest) && (recent <= newest))
        {
            medianValue = (oldest <= newest) ? oldest : newest;
        }
        else
        {
            medianValue = (oldest <= recent) ? oldest : recent;
        }
        // std::cout << "MEDIAN: OLD " << oldest << " " << recent << " " << newest << "  ==> " << medianValue << std::endl;
    };
    float value()
    {
        return medianValue;
    };

private:
    float oldest;
    float recent;
    float newest;
    float medianValue;
};




class FirstOrderMedianFilter
{
public:
    FirstOrderMedianFilter()
    {
        v[0] = v[1] = v[2] = v[3] = v[4] = 0;
        w[0] = w[1] = w[2] = w[3] = w[4] = 0;
        firstOrderMedianValue = 0;
        nextIndex = 0;
        lastValue = 0;
    };
    void init( float value )
    {
        v[0] = v[1] = v[2] = v[3] = v[4] = lastValue = value;
        firstOrderMedianValue = value;
    };
    void push( float value )
    {
        // rotate the values
        v[nextIndex] = value;
        w[nextIndex] = value - lastValue;
        lastValue = value;
        nextIndex = (nextIndex+1) % 5;
        float medianRawValue = median( v[0], v[1], v[2], v[3], v[4] );
        float medianDeltaV   = median( w[0], w[1], w[2], w[3], w[4] );
        firstOrderMedianValue = medianRawValue + 2*medianDeltaV;
    };
    float value()
    {
        return firstOrderMedianValue;
    };
private:
    // this median function Copyright © 2017 dlacko
    #define __swap__(a,b) __TEMP__ = b; b = a; a = __TEMP__;
    #define __sort__(a,b) if(a>b){ __swap__(a,b); }
    float median( float a, float b, float c, float d, float e )
    {
        float __TEMP__;
        __sort__(a,b);
        __sort__(d,e);  
        __sort__(a,c);
        __sort__(b,c);
        __sort__(a,d);  
        __sort__(c,d);
        __sort__(b,e);
        __sort__(b,c);
        // this last one is obviously unnecessary for the median
        //__sort__(d,e);
      
        return c;
    }
    #undef swap
    #undef sort
    // vN is the series of values
    float v[5];
    // wN is the series of first-order deltas
    float w[5];
    //
    int   nextIndex;
    float lastValue;
    float firstOrderMedianValue;
};

// function assumes 0 <= smoothness <= 1
// if smoothness == 0, return value is newValue
// if smoothness == 1, return value is oldValue (not recommended!)
// pick a value somewhere between....
float smoothFilter( float newValue, float oldValue, float smoothness = 0.9 )
{
    float newValueInfluence = 1.0 - smoothness;
    return newValueInfluence * newValue + oldValue * smoothness;
}

class Point2Dxy
{
public:
    float x, y;
    void rotate( float theta, Point2Dxy& source )
    {
        float ct = cos(theta);
        float st = sin(theta);
        x = source.x * ct - source.y * st;
        y = source.x * st + source.y * ct;
    };
    float getOriginDistance()
    {
        return sqrt( x*x + y*y );
    }
};

inline float dotProduct( const Point2Dxy& a, const Point2Dxy& b )
{
    return a.x * b.x + a.y * b.y;
}


namespace ev3dev {
class eopdv_sensor : public sensor
{
public:
    eopdv_sensor(address_type address) :
        sensor(address, { nxt_analog }),
        oldValue(6.0),
        eopdAddress(address)
    {
    }
    void init()
    {
        //
        // 1. Go to Device Browser >> ports >> in4 >> Set Mode
        // 2. change "auto" to "nxt-analog"
        //

        // std::cout << "eopdv_sensor device::connect():" << std::endl;
        // const std::map<std::string, std::set<std::string>> match(
        //         {{"address",{eopdAddress}}}
        //     );
        // std::cout << device::connect( "/sys/class/lego-port/", "port", match ) << std::endl;

        usleep( 100 * 1000 );

        std::cout << "eopdv_sensor set_mode(\"ANALOG-0\")" << std::endl;
        this->set_mode("ANALOG-0");
    }
    float valueInches()
    {
        // this conversion is for a matte white surface
        float newValue = 1 / ( 1.2016 * (5000.0 - this->value()) / 1000.0 + 0.08144 );
        // std::cout << "    EOPD     Vraw = " << this->value() << ",    inches = " << newValue << std::endl;
        return newValue;
        // oldValue = smoothFilter( newValue, oldValue, 0.9 );
        // return oldValue;
    }
private:
    float oldValue;
    address_type eopdAddress;
};
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class DifferentialBot (MVC:MODEL)
// * brief models the physical attributes of the robot
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class DifferentialBot
{
public:
    DifferentialBot() :
        irForwardOffsetCM(9.5),
        // usForwardOffsetCM(9.5),
        usPortSideOffsetCM(10.5),
        usStarboardOffsetCM(10.5),
        eopdStarboardOffsetInches(4.375), // 4 3/8 inces
        // LEGO bent beam interior obtuse angle is 126.87 degrees
        // the sensors are mounted on beams oriented like this:
        //            us         ir         us
        //             \\        ||        //   --- eopd
        //              \\=======||=======// ---
        // we use the angle between the us and the ir sensor bearings
        sinThetaBentBeam( sin( degrees2Rad( 36.87 ) ) ),
        cosThetaBentBeam( cos( degrees2Rad( 36.87 ) ) ),
        sinThetaEopdBeam( sin( degrees2Rad( 60.00 ) ) ),
        cosThetaEopdBeam( cos( degrees2Rad( 60.00 ) ) ),
        // sinThetaBentBeam( sin( degrees2Rad( 53.13 ) ) ),
        // cosThetaBentBeam( cos( degrees2Rad( 53.13 ) ) ),

        motorLeft(ev3dev::OUTPUT_B),
        motorRight(ev3dev::OUTPUT_C),
        irForward(ev3dev::INPUT_2),
        // usForward(ev3dev::INPUT_2),
        usPortSide(ev3dev::INPUT_1),
        usStarboard(ev3dev::INPUT_3),
        eopdStarboard(ev3dev::INPUT_4),
        axleTrack( cm2mils(14.4) ), // 18 holes * 8mm/hole
        tireDiameter( cm2mils(6.24) ) // 62.4 mm
    {
        tireCircumference = long( PI * float(tireDiameter) );
        // set the sendor modes
        try {
            usPortSide.distance_centimeters();
        }
        catch(const std::system_error& err) {
            std::cout << "Error while setting the mode of the port side ultrasonic sensor." << std::endl;
        }
        try {
            usStarboard.distance_centimeters();
        }
        catch(const std::system_error& err) {
            std::cout << "Error while setting the mode of the starboard ultrasonic sensor." << std::endl;
        }
        try {
            irForward.proximity();
        }
        catch(const std::system_error& err) {
            std::cout << "Error while setting the mode of the infrared proximity sensor." << std::endl;
        }
        // try {
        //     usForward.distance_centimeters();
        // }
        // catch(const std::system_error& err) {
        //     std::cout << "Error while setting the mode of the forward ultrasonic sensor." << std::endl;
        // }
        try {
            eopdStarboard.init();
        }
        catch(const std::system_error& err) {
            std::cout << "Error while setting the mode of the eopd sensor." << std::endl;
        }
        try {
            motorLeft.set_duty_cycle_sp(0);
        }
        catch(const std::system_error& err) {
            std::cout << "Error while initializing left motor." << std::endl;
        }
        try {
            motorRight.set_duty_cycle_sp(0);
        }
        catch(const std::system_error& err) {
            std::cout << "Error while initializing right motor." << std::endl;
        }
    };

    // this method should only ever be called by the BotManager
    void captureFrame()
    {
        static bool isFirstFrame = true;

        const float rightFactor = 2874.0 / 2891.0 ;
        const float leftFactor  = 1;

        if(isFirstFrame) std::cout << "DifferentialBot::captureFrame get motor positions" << std::endl;
        // on the old setup with  LEGO wheel 88516, diameter = 94.2mm = 3.7in ; R = 1.85in
        //          rightFactor = 2874.0 / 2891.0 ;
        motorLeftDegrees        = long( float(motorLeft.position()) * leftFactor );
        motorRightDegrees       = long( float(motorRight.position()) * rightFactor );

        // IR measurement returns as a percentage
        const float whiteBoxFactor = 0.7; // 100% is approximately 70cm/27in for a white box
        // read the distance sensors
        if(isFirstFrame) std::cout << "DifferentialBot::captureFrame get IR sensor data" << std::endl;
        irForwardFilter   .push( whiteBoxFactor * irForward.proximity(false) );
        // if(isFirstFrame) std::cout << "DifferentialBot::captureFrame get forward US sensor data" << std::endl;
        // usForwardFilter   .push( usForward.distance_centimeters(false) );
        if(isFirstFrame) std::cout << "DifferentialBot::captureFrame get port side US sensor data" << std::endl;
        usPortSideFilter  .push( usPortSide.distance_centimeters(false) );
        if(isFirstFrame) std::cout << "DifferentialBot::captureFrame get starboard US sensor data" << std::endl;
        usStarboardFilter .push( usStarboard.distance_centimeters(false) );
        if(isFirstFrame) std::cout << "DifferentialBot::captureFrame get eopd sensor data" << std::endl;
        eopdStarboardFilter.push( eopdStarboard.valueInches() );

        // std::cout << "    eopdStarboardFilter.value() = " << eopdStarboardFilter.value() << std::endl;

        float irForwardDistance     = cm2mils( irForwardFilter.value() + irForwardOffsetCM );
        // float usForwardDistance     = cm2mils( usForwardFilter.value() + usForwardOffsetCM );
        float usPortSideDistance    = cm2mils( usPortSideFilter.value() + usPortSideOffsetCM );
        float usStarboardDistance   = cm2mils( usStarboardFilter.value() + usStarboardOffsetCM );
        float eopdStarboardDistance = 1000 * ( eopdStarboardFilter.value() + eopdStarboardOffsetInches );

        irForwardPoint.x = irForwardDistance;
        irForwardPoint.y = 0;
        // usForwardPoint.x = usForwardDistance;
        // usForwardPoint.y = 0;
        usPortSidePoint.x = usPortSideDistance * cosThetaBentBeam;
        usPortSidePoint.y = usPortSideDistance * sinThetaBentBeam;
        usStarboardPoint.x = usStarboardDistance * cosThetaBentBeam;
        usStarboardPoint.y = - usStarboardDistance * sinThetaBentBeam;
        eopdStarboardPoint.x = eopdStarboardDistance * cosThetaEopdBeam;
        eopdStarboardPoint.y = - eopdStarboardDistance * sinThetaEopdBeam;

        isFirstFrame = false;
    };

    Point2Dxy& getForwardPoint()
    {
        return irForwardPoint;
        // return usForwardPoint;
    };

    Point2Dxy& getPortSidePoint()
    {
        return usPortSidePoint;
    };

    Point2Dxy& getStarboardPoint()
    {
        return usStarboardPoint;
    };

    Point2Dxy& getEopdStarboardPoint()
    {
        return eopdStarboardPoint;
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

    // float degreesPerInchOfTravel()
    // {
    //     const float milsPerInch = 1000.0;
    //     return 360.0 * milsPerInch / float(getTireCircumference());
    //     // // compensate for independently measured distance travelled
    //     // const float errorFactor = 39.0 / 38.0;
    //     // return 360.0 * milsPerInch * errorFactor / float(getTireCircumference());
    // };

private:

    ev3dev::motor motorLeft;
    ev3dev::motor motorRight;
    ev3dev::infrared_sensor   irForward;
    // ev3dev::ultrasonic_sensor usForward;
    ev3dev::ultrasonic_sensor usPortSide;
    ev3dev::ultrasonic_sensor usStarboard;
    ev3dev::eopdv_sensor      eopdStarboard;

    MedianFilter              irForwardFilter;
    // MedianFilter              usForwardFilter;
    MedianFilter              usPortSideFilter;
    MedianFilter              usStarboardFilter;
    // FirstOrderMedianFilter    eopdStarboardFilter;
    MedianFilter eopdStarboardFilter;

    const float    irForwardOffsetCM;
    // const float    usForwardOffsetCM;
    const float    usPortSideOffsetCM;
    const float    usStarboardOffsetCM;
    const float    eopdStarboardOffsetInches;
    const float    sinThetaBentBeam;
    const float    cosThetaBentBeam;
    const float    sinThetaEopdBeam;
    const float    cosThetaEopdBeam;

    Point2Dxy irForwardPoint;
    // Point2Dxy usForwardPoint;
    Point2Dxy usPortSidePoint;
    Point2Dxy usStarboardPoint;
    Point2Dxy eopdStarboardPoint;

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
        diffBot.getLeftMotor().set_position(0);
        diffBot.getRightMotor().set_position(0);

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
    float PhiDegrees() { return rad2Degrees(phi); };

private:

    char plotData[80][80];

    float             x, y, phi;
    float             axleTrack, tireDiameter;
    float             dw2ds;
    long              oldLeftDegrees, oldRightDegrees;
    DifferentialBot&  diffBot;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class Perceiver (MVC:CONTROLLER)
// * brief generic interface for various robot perception algorithms
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class Perceiver
{
public:
    Perceiver() { };
    virtual ~Perceiver() { };
    virtual void reset() { };
    virtual void update() = 0;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class PointObstaclePerceiver (MVC:CONTROLLER)
// * brief a perception algorithm that computes a single obstacle point
// *       from the three forward sensors
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class PointObstaclePerceiver : public Perceiver
{
public:
    PointObstaclePerceiver(
        DifferentialBot& theDiffBot,
        float maxAttentionRange
    ) :
        diffBot(theDiffBot),
        attentionRange(maxAttentionRange)
    {
        // nothing here
    };

    void update()
    {
        const int    POINT_COUNT = 3;

        Point2Dxy* p0 = &(diffBot.getForwardPoint());
        Point2Dxy* p1 = &(diffBot.getStarboardPoint());
        Point2Dxy* p2 = &(diffBot.getPortSidePoint());
        Point2Dxy* pointList[POINT_COUNT] = { p0, p1, p2 };

        int attentionPointCount = 0;
        obstaclePoint.x = 0;
        obstaclePoint.y = 0;
        for( int i=0; i<POINT_COUNT; i++ )
        {
            float x = pointList[i]->x;
            float y = pointList[i]->y;
            if( sqrt( x*x + y*y ) <= attentionRange )
            {
                obstaclePoint.x += x;
                obstaclePoint.y += y;
                ++attentionPointCount;
            }
        }
        obstaclePoint.x /= attentionPointCount;
        obstaclePoint.y /= attentionPointCount;
    };

    float getAttentionRange()
    {
        return attentionRange;
    };

    const Point2Dxy& getObstaclePoint()
    {
        return obstaclePoint;
    };

private:

    DifferentialBot& diffBot;
    float attentionRange;
    Point2Dxy obstaclePoint;

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
    virtual bool isDone() const = 0;
    virtual void stop() = 0;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorDriveStraightDifferentialPID (MVC:CONTROLLER)
// * brief drive straight in one direction
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorDriveStraightDifferentialPID : public Behavior
{
public:
    BehaviorDriveStraightDifferentialPID( 
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        long speedPercent = 50,
        long distanceMils = 0
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        targetDriveSpeed(speedPercent),
        targetDriveDistance(distanceMils),
        k_Pnumerator(7),
        k_Pdenomenator(5),
        k_Inumerator(1),
        k_Idenomenator(1),
        k_Dnumerator(1),
        k_Ddenomenator(4)
    {
        maxSignal = speedPercent * 1 / 5;
    };

    ~BehaviorDriveStraightDifferentialPID() { };

    void start()
    {
        ev3dev::motor motorLeft  = diffBot.getLeftMotor();
        ev3dev::motor motorRight = diffBot.getRightMotor();
        long rampTime = 50; // 250;

std::cout << "dpidControl start: get motor degrees" << std::endl;

        leftMotorInitValue  = diffBot.getLeftMotorDegrees();
        rightMotorInitValue = diffBot.getRightMotorDegrees();

std::cout << "dpidControl start: init motor settings" << std::endl;

        motorLeft
            .set_duty_cycle_sp(targetDriveSpeed)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake");

        motorRight
            .set_duty_cycle_sp(targetDriveSpeed)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake");

        eSum = 0;
        eOld = 0;
        tOld = usNow();

        x0 = odometer.X();
        y0 = odometer.Y();

std::cout << "dpidControl start: run motors" << std::endl;
std::cout << "x0="<<x0 << ", y0="<<y0 << ", odometer.X()="<<odometer.X()<<", odometer.Y()="<<odometer.Y() << std::endl;

        // now start the motors
        motorLeft.run_direct();
        motorRight.run_direct();
std::cout << "x0="<<x0 << ", y0="<<y0 << ", odometer.X()="<<odometer.X()<<", odometer.Y()="<<odometer.Y() << std::endl;
    };

    void update()
    {
        long e = ( diffBot.getLeftMotorDegrees() - leftMotorInitValue ) - 
                 ( diffBot.getRightMotorDegrees() - rightMotorInitValue );
        long D = ( e - eOld );

        eSum += e; // accumulate error over time

        long sP = k_Pnumerator * e / k_Pdenomenator;
        // long sI = k_Inumerator * eSum / k_Idenomenator;
        long sI = ortho_isqrt( k_Inumerator * eSum / k_Idenomenator );
        // long sD = k_Dnumerator * D / k_Ddenomenator;
        long sD = k_Dnumerator * ortho_square(D) / k_Ddenomenator;

        long signal = sP + sI + sD;

        if( signal > maxSignal )
        {
            signal = maxSignal;
        }
        else if( signal < -maxSignal )
        {
            signal = -maxSignal;
        }

        diffBot.getLeftMotor().set_duty_cycle_sp( targetDriveSpeed - signal );
        diffBot.getRightMotor().set_duty_cycle_sp( targetDriveSpeed + signal );

        std::cout << "BehaviorDriveStraightDifferentialPID e=" << e << ", sP=" << sP << ", eSum=" << eSum << ", sI=" << sI << ", D=" << D << ", sD=" << sD << ", signal=" << signal << std::endl;
std::cout << "x0="<<x0 << ", y0="<<y0 << ", odometer.X()="<<odometer.X()<<", odometer.Y()="<<odometer.Y() << std::endl;

        // if(sD){
        //     std::cout << "     " << " e=" << e << ", eOld=" << eOld << ", dt=" << dt << ", D=" << D << ", k_Dnumerator=" << k_Dnumerator << ", k_Ddenomenator=" << k_Ddenomenator << ", sD=" << sD << std::endl;
        //     // break;
        // }

        eOld = e;   // retain current error value
        // tOld = tN;  // retain current time value
    };

    bool isDone() const
    {
        std::cout << "x0="<<x0 << ", y0="<<y0 << ", odometer.X()="<<odometer.X()<<", odometer.Y()="<<odometer.Y() << std::endl;
        std::cout << "targetDriveDistance = " << targetDriveDistance <<",   actual distance = " << distance( x0, y0, odometer.X(), odometer.Y() ) << std::endl;
        return bool(targetDriveDistance) &&
               distance( x0, y0, odometer.X(), odometer.Y() ) >= targetDriveDistance;
    };

    void stop()
    {
        diffBot.getLeftMotor().stop();
        diffBot.getRightMotor().stop();
    };

protected:
    const long k_Pnumerator;
    const long k_Pdenomenator;
    const long k_Inumerator;
    const long k_Idenomenator;
    const long k_Dnumerator;
    const long k_Ddenomenator;

    DifferentialBot& diffBot;
    Odometer& odometer;

    long maxSignal;
    long targetDriveSpeed;
    long targetDriveDistance;
    long x0, y0;
    long leftMotorInitValue;
    long rightMotorInitValue;

    long eSum;
    long eOld;
    long tOld;
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorDriveStraightLineDifferentialPID (MVC:CONTROLLER)
// * brief drive straight in one direction on imaginary line
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorDriveStraightLineDifferentialPID : public Behavior
{
public:
    BehaviorDriveStraightLineDifferentialPID( 
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        long speedPercent = 50,
        long distanceMils = 0
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        targetDriveSpeed(speedPercent),
        targetDriveDistance(distanceMils),
        k_P(1.0 * 0.0254),
        k_I(0.0),
        k_D(5.0 * 0.0254)
    {
        maxSignal = speedPercent * 1 / 5;
    };

    ~BehaviorDriveStraightLineDifferentialPID() { };

    void start()
    {
        ev3dev::motor motorLeft  = diffBot.getLeftMotor();
        ev3dev::motor motorRight = diffBot.getRightMotor();
        long rampTime = 250;

std::cout << "line control start: get motor degrees" << std::endl;

        leftMotorInitValue  = diffBot.getLeftMotorDegrees();
        rightMotorInitValue = diffBot.getRightMotorDegrees();

std::cout << "line control start: init motor settings" << std::endl;

        motorLeft
            .set_duty_cycle_sp(targetDriveSpeed)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake");

        motorRight
            .set_duty_cycle_sp(targetDriveSpeed)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake");

        eOld = getError();
        eSum = eOld;
        tOld = usNow();

        x0 = odometer.X();
        y0 = odometer.Y();

std::cout << "line control start: run motors" << std::endl;

        // now start the motors
        motorLeft.run_direct();
        motorRight.run_direct();
    };

    virtual float getError()
    {
        return odometer.Y();
    }

    void update()
    {
        const float UMAX = 27.0;

        float e = getError();
        float D = ( e - eOld );

        eSum += e; // accumulate error over time

        float u = k_P*e + k_I*eSum + k_D*D;

        u /= UMAX;

        u = UMAX * u / sqrt( 1 + u*u );

        diffBot.getLeftMotor().set_duty_cycle_sp( targetDriveSpeed + u );
        diffBot.getRightMotor().set_duty_cycle_sp( targetDriveSpeed - u );

        // std::cout << " e=" << e << ", sP=" << (k_P*e) << ", eSum=" << eSum << ", sI=" << (k_I*eSum) << ", D=" << D << ", sD=" << (k_D*D) << ", u=" << u << std::endl;

        // std::cout << " e=" << e << ", sP=" << (k_P*e) << " I=" << eSum << ", sI=" << (k_I*eSum) << ", D=" << D << ", sD=" << (k_D*D) << ", u=" << u << std::endl;

        // if(sD){
        //     std::cout << "     " << " e=" << e << ", eOld=" << eOld << ", dt=" << dt << ", D=" << D << ", k_Dnumerator=" << k_Dnumerator << ", k_Ddenomenator=" << k_Ddenomenator << ", sD=" << sD << std::endl;
        //     // break;
        // }

        eOld = e;   // retain current error value
        // tOld = tN;  // retain current time value
    };

    bool isDone() const
    {
        return bool(targetDriveDistance) &&
               distance( x0, y0, odometer.X(), odometer.Y() ) >= targetDriveDistance;
    };

    void stop()
    {
        // diffBot.getLeftMotor().stop();
        // diffBot.getRightMotor().stop();
    };

protected:
    float k_P;
    float k_I;
    float k_D;

    DifferentialBot& diffBot;
    Odometer& odometer;

    long maxSignal;
    long targetDriveSpeed;
    long targetDriveDistance;
    float x0, y0;
    long leftMotorInitValue;
    long rightMotorInitValue;

    float eSum;
    float eOld;
    float tOld;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorFollowWall (MVC:CONTROLLER)
// * brief drive parallel to a wall on the right
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorFollowWall : public BehaviorDriveStraightLineDifferentialPID
{
public:
    BehaviorFollowWall( 
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        long speedPercent = 50,
        float theDesiredWallDistance = 6000
    ):
        desiredWallDistance(theDesiredWallDistance),
        BehaviorDriveStraightLineDifferentialPID( theDiffBot, theOdometer, speedPercent, 0 )
    {
        // k_I  = k_P / 2.0;
    };

    ~BehaviorFollowWall() { };

    float getError()
    {
        // the robot is a little under eight inches wide
        // the desired gap between the robot and the wall is about two inches
        // so the desired lateral distance from center is 4 (half width) + 2 = 6 inches
        // the sensor is mounted at a 30 degree angle from lateral, so the
        // desired distance to the wall in the direction of the sensor is
        // given by the equation:
        //       Ds * cos30 = Dw    =>   Ds = Dw / cos30
        const float cos30 = 0.866;
        // const float cos37 = 0.800;
        const float MAX_DISTANCE = 12000;

        float desiredSensorDistance = desiredWallDistance / cos30;
        // float desiredSensorDistance = desiredWallDistance / cos37;

        Point2Dxy& wallPoint = diffBot.getEopdStarboardPoint();

        // Point2Dxy& wallPoint = diffBot.getStarboardPoint();

        float actualDistance = sqrt( wallPoint.x * wallPoint.x + wallPoint.y * wallPoint.y );

        std::cout << "   wallPoint.x = " << wallPoint.x << "   wallPoint.y = " << wallPoint.y << std::endl;
        std::cout << "   desiredWallDistance = " << desiredWallDistance << "   desiredSensorDistance = " << desiredSensorDistance << "   actualDistance = " << actualDistance << std::endl;

        if( ! (actualDistance < MAX_DISTANCE ) )
        {
            actualDistance = MAX_DISTANCE;
        }
        // positive error if robot is too far away/left
        // negative error if robot is too far right
        return actualDistance - desiredSensorDistance;
    };

    bool isDone() const
    {
        return false;
    };

private:
    float desiredWallDistance;
};



// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorAvoidForwardObstacle (MVC:CONTROLLER)
// * brief turn left to avoid an obstacle in front of the robot
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorAvoidForwardObstacle : public Behavior
{
public:
    BehaviorAvoidForwardObstacle(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer
    ):
        diffBot(theDiffBot),
        odometer(theOdometer)
        // k_P(0.016),
        // k_I(0.0),
        // k_D(0.050),
        // targetDistanceMILS( 10 * 1000.0 ),
        // sumError(0.0)
    {
    };

    void start()
    {
        // Point2Dxy& obstaclePoint = diffBot.getForwardPoint();
        // float distance = obstaclePoint.getOriginDistance();
        // oldError = targetDistanceMILS - distance;
        // sumError = 0.0;
    };

    void update()
    {
        std::cout << "  BehaviorAvoidForwardObstacle turns hard to port side" << std::endl;
        diffBot.getLeftMotor().set_duty_cycle_sp( 0 );
        diffBot.getRightMotor().set_duty_cycle_sp( 80 );
    };

    // void OLD_update()
    // {
    //     const float UMAX = 30.0;
    //     static float smoothDeltaError = 0;

    //     Point2Dxy& forwardPoint = diffBot.getForwardPoint();
    //     float distance = forwardPoint.getOriginDistance();
    //     Point2Dxy& starboardPoint = diffBot.getStarboardPoint();
    //     distance = std::min( distance, starboardPoint.getOriginDistance() );
    //     // Point2Dxy& eopdPoint = diffBot.getEopdStarboardPoint();
    //     // distance = std::min( distance, eopdPoint.getOriginDistance() );
    //     //
    //     // trigger distance is nine inches
    //     // critical minimum distance is five inches
    //     // exit distance is ten inches
    //     //
    //     float error   = targetDistanceMILS - distance;
    //     error   = std::max( 0.0f, error );
    //     float eDelta  = error - oldError;
    //     sumError     += error;

    //     smoothDeltaError = smoothFilter( eDelta, smoothDeltaError, 0.65 );

    //     float u = k_P * error + k_I * sumError + k_D * smoothDeltaError;

    //     std::cout << "e="<<error<< ", de="<<eDelta<<", deSmooth="<<smoothDeltaError<< ", uRaw="<<u ;

    //     u /= UMAX;
    //     u = UMAX * u / sqrt( 1 + u*u );

    //     std::cout << ", uSig="<<u<< std::endl;

    //     diffBot.getLeftMotor().set_duty_cycle_sp( 30 - 2*u );
    //     diffBot.getRightMotor().set_duty_cycle_sp( 30 + u );

    //     oldError = error;
    // };

    void stop() { };

    bool isDone() const
    {
        return false;
    };

protected:
    // const float k_P;
    // const float k_I;
    // const float k_D;
    // const float targetDistanceMILS;

    DifferentialBot& diffBot;
    Odometer& odometer;
    // float oldError;
    // float sumError;
};



// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorDriveCircle (MVC:CONTROLLER)
// * brief drive around the origin in a wide circle of a specific radius
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorDriveCircle : public Behavior
{
public:
    BehaviorDriveCircle(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        float circleRadiusMils,
        long  speedPercent = 50,
        float distanceMils= 0
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        targetRadius(circleRadiusMils),
        targetDriveSpeed(speedPercent),
        targetDriveDistance(distanceMils),
        k_P(17), // (0.2),
        k_I(0.0),
        k_D(0.0)
    {
        maxSignal = std::min( 85 - targetDriveSpeed, targetDriveSpeed - 15 );
    };

    ~BehaviorDriveCircle() { };

    void start()
    {
        ev3dev::motor motorLeft  = diffBot.getLeftMotor();
        ev3dev::motor motorRight = diffBot.getRightMotor();
        long rampTime = 250;

        leftMotorInitValue  = diffBot.getLeftMotorDegrees();
        rightMotorInitValue = diffBot.getRightMotorDegrees();

        float wheelOffset = diffBot.getAxleTrack() / 2;
        targetDriveSpeedLeft  = targetDriveSpeed * (targetRadius + wheelOffset) / targetRadius;
        targetDriveSpeedRight = targetDriveSpeed * (targetRadius - wheelOffset) / targetRadius;


std::cout << "targetRadius=" << targetRadius 
        << ", AxleTrack=" << diffBot.getAxleTrack() 
        << ", wheelOffset=" << wheelOffset 
        << ", targetDriveSpeed=" << targetDriveSpeed 
        << ", targetDriveSpeedLeft=" << targetDriveSpeedLeft 
        << ", targetDriveSpeedRight=" << targetDriveSpeedRight 
        << std::endl;

        motorLeft
            .set_duty_cycle_sp(targetDriveSpeedLeft)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake");

        motorRight
            .set_duty_cycle_sp(targetDriveSpeedRight)
            .set_ramp_up_sp(rampTime)
            .set_stop_action("brake");

        eSum = 0;
        eOld = 0;
        dOld = 0;
        tOld = usNow();

        driveDistance = 0;
        xOld = odometer.X();
        yOld = odometer.Y();

        // now start the motors
        motorLeft.run_direct();
        motorRight.run_direct();
    };

    void update()
    {
        float originDistance = distance( 0, 0, odometer.X(), odometer.Y() );
        float e = originDistance - targetRadius;
        float D = ( e - eOld );
        float a = D - dOld;

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


        diffBot.getLeftMotor().set_duty_cycle_sp( targetDriveSpeedLeft + u );
        diffBot.getRightMotor().set_duty_cycle_sp( targetDriveSpeedRight - u );

        std::cout << std::fixed << std::setprecision(1) 
            << "  x=" << odometer.X() 
            << ", y=" << odometer.Y() 
            << ", od=" << originDistance 
            << ", e=" << e
            << ", sP=" << (k_P*e)
            << ", eSum=" << eSum
            << ", sI=" << (k_I*eSum)
            << ", D=" << D
            << ", sD=" << (k_D*D)
            << ", u=" << u
            << ", a=" << a
            << std::endl;

        eOld = e;   // retain current error value
        dOld = D;

        driveDistance += distance( xOld, yOld, odometer.X(), odometer.Y() );
        xOld = odometer.X();
        yOld = odometer.Y();
    };

    bool isDone() const
    {
        return bool(targetDriveDistance) && driveDistance >= targetDriveDistance;
    };

    void stop()
    {
        diffBot.getLeftMotor().stop();
        diffBot.getRightMotor().stop();
    };

private:
    const float k_P;
    const float k_I;
    const float k_D;

    DifferentialBot& diffBot;
    Odometer& odometer;

    long maxSignal;
    float targetRadius;
    long targetDriveSpeed;
    float targetDriveSpeedLeft;
    float targetDriveSpeedRight;
    float targetDriveDistance;
    float xOld, yOld;
    long leftMotorInitValue;
    long rightMotorInitValue;

    float eSum;
    float eOld;
    float tOld;
    float dOld;
    float driveDistance;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorPivotTurn (MVC:CONTROLLER)
// * brief make the robot pivot around one wheel
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorPivotTurn : public Behavior
{
public:
    BehaviorPivotTurn(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        ev3dev::motor& thePivotMotor,
        ev3dev::motor& theActiveMotor,
        float degrees = 90.0,
        bool isForward = true
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        pivotMotor(thePivotMotor),
        activeMotor(theActiveMotor),
        turnDegrees(degrees)
    {
        dutyCycle = isForward ? 25 : -25;
    };

    ~BehaviorPivotTurn() { };

    void start()
    {
        pivotMotor.stop();
        activeMotor
            .set_duty_cycle_sp(dutyCycle)
            .set_ramp_up_sp(250);
        phiStart = odometer.PhiDegrees();
        isRampDownStarted = false;
        activeMotor.run_direct();
    };
    void update()
    {
        if( !isRampDownStarted && 
            fabs( phiStart - odometer.PhiDegrees() ) >= (turnDegrees-10) )
        {
            isRampDownStarted = true;
std::cout << " RAMP DOWN!" << std::endl;
            if( fabs(dutyCycle) > 15 )
            {
                int newDutyCycle = 15;
                if( dutyCycle < 0 )
                {
                    newDutyCycle = -15;
                }
                activeMotor.set_duty_cycle_sp(newDutyCycle);
            }
        }
    };
    bool isDone() const
    {
        std::cout << " PIVOT TURN   phiStart = " << phiStart << " -- PHI (degrees) = " << odometer.PhiDegrees() << std::endl;
        return fabs( phiStart - odometer.PhiDegrees() ) >= turnDegrees;
    };
    void stop()
    {
        activeMotor.stop();
    };
private:
    DifferentialBot& diffBot;
    Odometer& odometer;
    ev3dev::motor& pivotMotor;
    ev3dev::motor& activeMotor;
    int dutyCycle;
    float phiStart;
    float turnDegrees;
    bool isRampDownStarted;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorSpinTurn (MVC:CONTROLLER)
// * brief make the robot spin around its center
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorSpinTurn : public Behavior
{
public:
    BehaviorSpinTurn(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        ev3dev::motor& theReverseMotor,
        ev3dev::motor& theForwardMotor,
        float degrees = 90.0
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        reverseMotor(theReverseMotor),
        forwardMotor(theForwardMotor),
        turnDegrees(degrees)
    {
        dutyCycle = 15;
    };

    ~BehaviorSpinTurn() { };

    void start()
    {
        reverseMotor
            .set_duty_cycle_sp(-dutyCycle)
            .set_ramp_up_sp(250);
        forwardMotor
            .set_duty_cycle_sp(dutyCycle)
            .set_ramp_up_sp(250);
        phiStart = odometer.PhiDegrees();
        isRampDownStarted = false;
        reverseMotor.run_direct();
        forwardMotor.run_direct();
    };
    void update()
    {
//         if( !isRampDownStarted && 
//             fabs( phiStart - odometer.PhiDegrees() ) >= (turnDegrees-10) )
//         {
//             isRampDownStarted = true;
// std::cout << " RAMP DOWN!" << std::endl;
//             reverseMotor.set_duty_cycle_sp(-dutyCycle);
//             forwardMotor.set_duty_cycle_sp(dutyCycle);
//         }
    };
    bool isDone() const
    {
        std::cout << " SPIN TURN phiStart = " << phiStart << " -- PHI (degrees) = " << odometer.PhiDegrees() << std::endl;
        return fabs( phiStart - odometer.PhiDegrees() ) >= turnDegrees;
    };
    void stop()
    {
        reverseMotor.stop();
        forwardMotor.stop();
    };
private:
    DifferentialBot& diffBot;
    Odometer& odometer;
    ev3dev::motor& reverseMotor;
    ev3dev::motor& forwardMotor;
    int dutyCycle;
    float phiStart;
    float turnDegrees;
    bool isRampDownStarted;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorCircleTurn (MVC:CONTROLLER)
// * brief make the robot turn in a circle
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorCircleTurn : public Behavior
{
public:
    BehaviorCircleTurn(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        float theTurnRadiusMils
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        turnRadiusMils(theTurnRadiusMils)
    {
    };

    ~BehaviorCircleTurn() { };

    void start()
    {
        ev3dev::motor* fastMotor;
        ev3dev::motor* slowMotor;
        ev3dev::motor& leftMotor(diffBot.getLeftMotor());
        ev3dev::motor& rightMotor(diffBot.getRightMotor());
        float absTurnRadius = turnRadiusMils;

        // if turnRadius is POSITIVE, robot turns RIGHT
        if( turnRadiusMils > 0 )
        {
            fastMotor = &leftMotor;
            slowMotor = &rightMotor;
            std::cout << "BehaviorCircleTurn RIGHT radius " << absTurnRadius << std::endl;
        }
        // if turnRadius is NEGATIVE, robot turns LEFT
        else
        {
            absTurnRadius = -1 * turnRadiusMils;
            fastMotor = &rightMotor;
            slowMotor = &leftMotor;
            std::cout << "BehaviorCircleTurn LEFT radius " << absTurnRadius << std::endl;
        }

        float halfTrack = diffBot.getAxleTrack() / 2.0;
        float dutyCycle = 50.0;
        float fastCycle = dutyCycle * ( absTurnRadius + halfTrack ) / absTurnRadius;
        float slowCycle = dutyCycle * ( absTurnRadius - halfTrack ) / absTurnRadius;

        (*fastMotor)
            .set_duty_cycle_sp(fastCycle)
            .set_ramp_up_sp(100);
        (*slowMotor)
            .set_duty_cycle_sp(slowCycle)
            .set_ramp_up_sp(100);

        (*fastMotor).run_direct();
        (*slowMotor).run_direct();
    };
    void update()
    {
    };
    bool isDone() const
    {
        return false;
    };
    void stop()
    {
        diffBot.getLeftMotor().stop();
        diffBot.getRightMotor().stop();
    };
private:
    DifferentialBot& diffBot;
    Odometer& odometer;
    float turnRadiusMils;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorBrake (MVC:CONTROLLER)
// * brief stop all motors
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorBrake : public Behavior
{
public:
    BehaviorBrake( DifferentialBot& theDiffBot ) : diffBot(theDiffBot) { };
    ~BehaviorBrake() { };
    void start()
    {
        diffBot.getLeftMotor()
            .set_stop_action("brake")
            .stop();
        diffBot.getRightMotor()
            .set_stop_action("brake")
            .stop();
    };
    void update() { };
    bool isDone() const { return true; };
    void stop() { };
private:
    DifferentialBot& diffBot;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorWaitForBackButton (MVC:CONTROLLER)
// * brief wait for user to press the ESC button
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorWaitForBackButton : public Behavior
{
public:
    BehaviorWaitForBackButton( DifferentialBot& theDiffBot ) : diffBot(theDiffBot) { };
    ~BehaviorWaitForBackButton() { };
    void start() { };
    void update() { };
    bool isDone() const { ev3dev::button::back.pressed(); };
    void stop() { };
private:
    DifferentialBot& diffBot;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorGoToGoal (MVC:CONTROLLER)
// * brief drive to a goal point
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorGoToGoal : public Behavior
{
public:
    BehaviorGoToGoal(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        PointObstaclePerceiver* pop,
        long xTarget,
        long yTarget
    ):
        diffBot(theDiffBot),
        odometer(theOdometer),
        pointObstaclePerceiver(pop),
        xGoal(xTarget),
        yGoal(yTarget),
        KP(7.0),
        KI(0.0),
        KD(0.0),
        rampUpSpeed(250)
    {
    };

    ~BehaviorGoToGoal() { };

    void start()
    {
        std::cout << "goto goal START" << std::endl;
        ev3dev::motor& leftMotor  = diffBot.getLeftMotor();
        ev3dev::motor& rightMotor = diffBot.getRightMotor();

        dutyCycleLeft = 30;
        dutyCycleRight = 30;
        maxSignal = 22.0;
        eSum = 0.0;
        eOld = 0.0;

        leftMotor
            .set_duty_cycle_sp(dutyCycleLeft)
            .set_ramp_up_sp(rampUpSpeed)
            .set_stop_action("coast");

        rightMotor
            .set_duty_cycle_sp(dutyCycleRight)
            .set_ramp_up_sp(rampUpSpeed)
            .set_stop_action("coast");

        isRampDownStarted = false;

        leftMotor.run_direct();
        rightMotor.run_direct();

        distanceToGoal = 9999;

    };

    void update()
    {
        float xRobot = odometer.X();
        float yRobot = odometer.Y();

        distanceToGoal = distance( xGoal, yGoal, xRobot, yRobot );

// std::cout << "   xGoal = " << xGoal << " , yGoal = " << yGoal
//           << "   xRobot = " << xRobot << " , yRobot = " << yRobot
//           << "   distanceToGoal = " << distanceToGoal << std::endl;

        if( !isRampDownStarted && distanceToGoal < 4000 )
        {
            isRampDownStarted = true;
            dutyCycleLeft = 15;
            dutyCycleRight = 15;
            maxSignal = 5.0;
            std::cout << " RAMP DOWN!" << std::endl;
        }
        if( rampUpSpeed > 20 )
        {
            rampUpSpeed -= 10;
        }

        float dx = xGoal - xRobot;
        float dy = yGoal - yRobot;

        float phiGoal = atan2( dy, dx );

        // // ===========================================================
        // // HYBRIDIZATION of behavior: blend with avoiding obstacle...
        // if( pointObstaclePerceiver )
        // {
        //     Point2Dxy& obstacle = pointObstaclePerceiver->getObstaclePoint();
        //     // obstacle is relative to the robot's frame of reference
        //     // and (0,0) means there is no obstacle
        //     if( obstacle.x || obstacle.y )
        //     {
        //         float phiObstacle = atan2( obstacle.y, obstacle.x );
        //         float avoidanceAngles[2] = [ PI/2, -PI/2 ];
        //         Point2Dxy goalVector = { dx, dy };
        //         float selectedAngle = 9e99;
        //         for( int i=0; i<2; i++ )
        //         {
        //             Point2Dxy avoidanceVector = obstacle.rotate( avoidanceAngles[i] );
        //             if( dotProduct(avoidanceVector,goalVector) >= 0 )
        //             {
        //                 selectedAngle = avoidanceAngles[i];
        //             }
        //         }
        //         if( selectedAngle != 9e99 )
        //         {
        //             float obstacleDistance = distance( obstacle.x*obstacle.x + obstacle.y*obstacle.y );
        //             float maxRange         = pointObstaclePerceiver.getAttentionRange();
        //             float weightAvoidance  = ( maxRange - obstacleDistance ) / maxRange;
        //             float weightGoToGoal   = 1 - weightAvoidance;
        //             float xHybrid = weightGoToGoal*cos(phiGoal) + weightAvoidance*cos(selectedAngle);
        //             float yHybrid = weightGoToGoal*sin(phiGoal) + weightAvoidance*sin(selectedAngle);
        //             //
        //             // HERE WE OVERRIDE THE GOAL DIRECTION WITH THE NEW HYBRID DIRECTION
        //             //
        //             phiGoal = atan2( yHybrid, xHybrid );
        //         }
        //     }
        // }

        float eRad = ( phiGoal - odometer.PhiRadians() );

        // it's easier (for me) to consider headings in degrees
        float e = rad2Degrees( atan2( sin(eRad), cos(eRad) ) );
        eSum += e;
        float de = e - eOld;

        float signal = KP * e; //  + KI * eSum + KD * de;
        long iSignal = std::min( maxSignal, std::max( -maxSignal, signal ) );

// std::cout << "    phiGoal = " << rad2Degrees(phiGoal) << ", odometer.PhiDegrees() = " << odometer.PhiDegrees() << ", e = " << e << ", signal = " << iSignal << std::endl;

        diffBot.getLeftMotor()
            .set_ramp_up_sp(rampUpSpeed)
            .set_duty_cycle_sp( dutyCycleLeft - iSignal );
        diffBot.getRightMotor()
            .set_ramp_up_sp(rampUpSpeed)
            .set_duty_cycle_sp( dutyCycleRight + iSignal );
    };

    bool isDone() const
    {
        // return( idist( xGoal, yGoal, long(odometer.X()), long(odometer.Y()) ) < 500 );
        return ( distanceToGoal < 999 );
    };

    void stop()
    {
        diffBot.getRightMotor().stop();
        diffBot.getLeftMotor().stop();
    };

protected:
    DifferentialBot& diffBot;
    Odometer& odometer;
    PointObstaclePerceiver* pointObstaclePerceiver;
    int dutyCycleLeft;
    int dutyCycleRight;
    int rampUpSpeed;
    float maxSignal;
    long xGoal, yGoal;
    long distanceToGoal;
    bool isRampDownStarted;
    float eSum;
    float eOld;
    const float KP;
    const float KI;
    const float KD;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BehaviorWideCircle (MVC:CONTROLLER)
// * brief drive in a wide circle
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BehaviorWideCircle : public BehaviorGoToGoal
{
public:
    BehaviorWideCircle(
        DifferentialBot& theDiffBot,
        Odometer& theOdometer,
        PointObstaclePerceiver* pop,
        float circleRadiusMils,
        float xCircleCenter = 0,
        float yCircleCenter = 0
    ):
        BehaviorGoToGoal( theDiffBot, theOdometer, pop, 0, 0 ),
        radius(circleRadiusMils),
        xCenter(xCircleCenter),
        yCenter(yCircleCenter)
    {
        leadAngle =  ( 0.25 * theDiffBot.getAxleTrack() / radius );
        std::cout << "leadAngle = " << leadAngle << " radians,  " << rad2Degrees(leadAngle) << " degrees,  " << std::endl;
    };
    ~BehaviorWideCircle()
    {
        // nothing here
    };
    void start()
    {
        distanceTravelled = 0;
        xOld = odometer.X();
        yOld = odometer.Y();
        BehaviorGoToGoal::start();
        // 
        isRampDownStarted = true;

    };
    void update()
    {
        // calculate the dynamic goal point
        float xRobot = odometer.X();
        float yRobot = odometer.Y();
        float dx = xRobot - xCenter;
        float dy = yRobot - yCenter;
        float robotAngle = atan2( dy, dx );
        float targetAngle = robotAngle - leadAngle;
        xGoal = xCenter + radius * cos( targetAngle );
        yGoal = yCenter + radius * sin( targetAngle );
        // calculate distance travelled
        distanceTravelled += distance( xOld, yOld, xRobot, yRobot );
        xOld = xRobot;
        yOld = yRobot;

        float d = distance( xCenter, yCenter, xRobot, yRobot );
        // std::cout << std::fixed << std::setprecision(1) 
        //     << " d=" << d
        //     << ", error=" << (d-radius)
        //     << ", xRobot=" << xRobot
        //     << ", yRobot=" << yRobot
        //     << ", xGoal=" << xGoal
        //     << ", yGoal=" << yGoal
        //     << ", robotAngle=" << robotAngle
        //     << ", targetAngle=" << targetAngle
        //     << std::endl;

        // run the parent update behavior
        BehaviorGoToGoal::update();
    };
    bool isDone() const
    {
        return( distanceTravelled >= 2 * PI * radius );
    };
    // use inherited stop()

protected:
    float xCenter, yCenter, radius;
    float xOld, yOld, distanceTravelled;
    float leadAngle;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// * class BotManager (MVC:CONTROLLER)
// * brief manages frames and selects behaviors
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class BotManager
{
public:
    BotManager():
       diffBot(),
       odometer(diffBot)
    {
        odometer.reset();
    };

    void run()
    {
        // do something here
        long speedPercent = 25;
        long distanceMils = 13 * 1000;

        std::vector <Behavior*> behaviorSequence;
        std::vector <Perceiver*> perceiverList;

        // BehaviorDriveStraightDifferentialPID driveForward(
        //     diffBot,
        //     odometer,
        //     speedPercent,
        //     distanceMils
        // );
        // BehaviorPivotTurn pivotLeftForward(
        //     diffBot,
        //     odometer,
        //     diffBot.getLeftMotor(),
        //     diffBot.getRightMotor()
        // );
        // behaviorSequence.push_back( &driveForward );
        // behaviorSequence.push_back( &pivotLeftForward );
        // behaviorSequence.push_back( &pivotLeftForward );

        // BehaviorGoToGoal gotoWP1( diffBot, odometer, 91000,     0 );
        // BehaviorGoToGoal gotoWP2( diffBot, odometer, 91000, -52000 );
        // BehaviorGoToGoal gotoWP3( diffBot, odometer,     0, -52000 );
        // BehaviorGoToGoal gotoWP4( diffBot, odometer,     0,     0 );

        // BehaviorGoToGoal gotoWP1( diffBot, odometer, 22000,      0 );
        // BehaviorGoToGoal gotoWP2( diffBot, odometer, 22000, -26000 );
        // BehaviorGoToGoal gotoWP3( diffBot, odometer,     0, -26000 );
        // BehaviorGoToGoal gotoWP4( diffBot, odometer,     0,      0 );

        // behaviorSequence.push_back( &gotoWP1 );
        // behaviorSequence.push_back( &gotoWP2 );
        // behaviorSequence.push_back( &gotoWP3 );
        // behaviorSequence.push_back( &gotoWP4 );

        // BehaviorDriveStraightLineDifferentialPID driveForwardA( diffBot, odometer, 30, 39000 );
        // behaviorSequence.push_back( &driveForwardA );

        // BehaviorDriveStraightDifferentialPID driveForwardA( diffBot, odometer, 30, 91000 );
        // BehaviorDriveStraightDifferentialPID driveForwardB( diffBot, odometer, 30, 52000 );
        // BehaviorSpinTurn  spinRightTurn( diffBot, odometer, diffBot.getRightMotor(), diffBot.getLeftMotor() );
        // BehaviorPivotTurn pivotRightTurn( diffBot, odometer, diffBot.getRightMotor(), diffBot.getLeftMotor(), 88 );

        // behaviorSequence.push_back( &driveForwardA );
        // behaviorSequence.push_back( &pivotRightTurn );
        // behaviorSequence.push_back( &driveForwardB );
        // behaviorSequence.push_back( &pivotRightTurn );
        // behaviorSequence.push_back( &driveForwardA );
        // behaviorSequence.push_back( &pivotRightTurn );
        // behaviorSequence.push_back( &driveForwardB );
        // behaviorSequence.push_back( &pivotRightTurn );

        // BehaviorDriveStraightDifferentialPID driveForward65in(
        //     diffBot,
        //     odometer,
        //     20,
        //     65536
        // );
        // behaviorSequence.push_back( &driveForward65in );
        // BehaviorWaitForBackButton waitForbackButton(diffBot);
        // behaviorSequence.push_back( &waitForbackButton );

        PointObstaclePerceiver pop( diffBot, 14 * 1000 );
        perceiverList.push_back( &pop );

        float radius = 39 * 1000; 

        BehaviorGoToGoal gotoWP1( diffBot, odometer, &pop, 0, - radius );
        behaviorSequence.push_back( &gotoWP1 );

        // BehaviorDriveCircle driveCircle( diffBot, odometer, radius, 50, 0.35 * 2*PI*radius );
        // behaviorSequence.push_back( &driveCircle );

        BehaviorWideCircle driveCircle( diffBot, odometer, &pop, radius, 0, 0 );
        behaviorSequence.push_back( &driveCircle );

        unsigned long tStart = usNow();
        long frameCount = 0;

        float yMax = odometer.Y();

        // for( int b=0; b<behaviorSequence.size(); ++b )
        // {
        //     Behavior& behavior = *behaviorSequence[b];
        for( auto b : behaviorSequence )
        {
            Behavior& behavior = *b;
            behavior.start();
            do
            {
                ++frameCount;

                diffBot.captureFrame();
                if( yMax < odometer.Y() ) {
                    yMax = odometer.Y();
                }
                odometer.update();
                behavior.update();
                usleep( 20 * 1000 );
            }
            while( !behavior.isDone() );
            behavior.stop();
        }

        if(frameCount)
        {
            unsigned long usTotal = usNow() - tStart;
            unsigned long dtAverage = usTotal / frameCount;

            std::cout << "   x=" << odometer.X() << "   y=" << odometer.Y() << "   usTotal=" << usTotal << "   frameCount=" << frameCount << "   dtAverage=" << dtAverage << "us" << std::endl;
            // std::cout << "   yMax=" << yMax << "   usTotal=" << usTotal << "   frameCount=" << frameCount << "   dtAverage=" << dtAverage << "us" << std::endl;
        }
    };
private:
    DifferentialBot diffBot;
    Odometer odometer;
};



class BehaviorState;

// class GuardCondition
// {
// public:
//     GuardCondition(
//         DifferentialBot& diffBot,
//         Odometer& odometer
//     ) :
//         robot(diffBot),
//         odo(odometer)
//     {
//         //
//     };
//     virtual
// protected:
//     DifferentialBot& robot;
//     Odometer& odo;

// }


typedef std::function<bool ()> GuardCondition;

class ModeTransition
{
public:
    ModeTransition(
        BehaviorState* theBehaviorState,
        GuardCondition theGuardCondition
    ) :
        indicatedBehaviorState(theBehaviorState),
        guardCondition(theGuardCondition)
    {
        //
    };
    bool isIndicated()
    {
        // std::cout << "call guardCondition()" << std::endl;
        return guardCondition();
    };
    // void reset() { }; // can be used to modify/update the observer (robot state info)
    BehaviorState* getBehaviorState()
    {
        return indicatedBehaviorState;
    };
protected:
    GuardCondition guardCondition;
    BehaviorState* indicatedBehaviorState;
private:
    ModeTransition(ModeTransition& mt){};
};


class BehaviorState
{
public:
    BehaviorState(
        const char* theName,
        Behavior& theBehavior
    ) :
        myName(theName),
        myBehavior(theBehavior)
    {
    };
    void pushTransition( ModeTransition* theTransition )
    {
        modeTransitions.push_back(theTransition);
    };
    std::string getName()
    {
        return myName;
    };
    Behavior* getBehavior()
    {
        return &myBehavior;
    };
    BehaviorState* checkForNewBehaviorState()
    {
        for( auto iter = modeTransitions.begin(); iter != modeTransitions.end(); iter++ )
        {
            ModeTransition* modeTransition = *iter;
            if( modeTransition->isIndicated() )
            {
                // std::cout << "mode transition is indicated!" << std::endl;
                return modeTransition->getBehaviorState();
            }
        }
        return nullptr;
    };

private:
    std::vector<ModeTransition*> modeTransitions;
    Behavior& myBehavior;
    std::string myName;
};


void eopdFilterTest()
{
    std::cout << "START TEST SECTION" << std::endl;
    // int value = -1;
    float value = -1;
    std::cout << "make instance of eopdv" << std::endl;
    ev3dev::eopdv_sensor eopd4(ev3dev::INPUT_4);


    std::cout << "eopd4.init();" << std::endl;
    eopd4.init();



    std::cout << "eopd driver_name:" << std::endl;
    std::cout << eopd4.driver_name() << std::endl;

    std::cout << "eopd modes:" << std::endl;
    ev3dev::mode_set eopdModes = eopd4.modes();
    for( ev3dev::mode_set::iterator it=eopdModes.begin(); it != eopdModes.end(); it++ )
    {
        std::cout << " " << *it;
    }
    std::cout << std::endl;

    std::cout << "start for-loop" << std::endl;
    for( int i=0; i<999; i++ )
    {
        // std::cout << "   -- get eopd4.value()" << std::endl;
        // int newValue = eopd4.value();
        float newValue = eopd4.valueInches();
        if( value != newValue )
        {
            value = newValue;
            std::cout << value << std::endl;
        }
    }
}


void BehaviorTest(
    Behavior& behavior,
    DifferentialBot& diffBot,
    Odometer& odometer )
{
    try
    {
        int approxSeconds = 5;
        int tickPeriodMS  = 20;
        int tickPeriodUS  = tickPeriodMS * 1000;
        // std::cout << "tickPeriodUS is " << tickPeriodUS << "us" << std::endl;
        int numIterations = approxSeconds * 1000 / tickPeriodMS;
        int smoothSleepTimeUS = 0;

        behavior.start();

        for( int i=0; i<numIterations && !ev3dev::button::back.pressed(); ++i )
        {
            unsigned frameStartTime = usNow();
            // get sensor data and compute new robot state values
            diffBot.captureFrame();
            odometer.update();

            // update the robot using the current behavior
            behavior.update();

            // calculate how long to sleep 
            unsigned elapsedTime = usNow() - frameStartTime;
            if( tickPeriodUS > elapsedTime )
            {
                // std::cout << "sleep for " << tickPeriodUS - elapsedTime << "us" << std::endl;
                smoothSleepTimeUS = smoothFilter( tickPeriodUS - elapsedTime, smoothSleepTimeUS, 0.95 );
                usleep( tickPeriodUS - elapsedTime );
            }
        }
        std::cout<< "smoothSleepTimeUS="<<smoothSleepTimeUS << std::endl;
    }
    catch(const std::system_error& e)
    {
        std::cout << "MazeSolver caught a system_error with code " << e.code() 
                  << " meaning " << e.what() << '\n';
    }

    BehaviorBrake brake(diffBot);
    brake.start();
}

class MazeSolver
{
public:
    MazeSolver() :
       diffBot(),
       odometer(diffBot)
    {
        odometer.reset();
    };

    void run()
    {
        const float FORWARD_FAR_THRESHOLD = 9500.0;
        const float STARBOARD_FAR_THRESHOLD_LEFT_TURN = 8800.0;
        const float STARBOARD_NEAR_THRESHOLD_LEFT_TURN = 8000.0;
        const float STARBOARD_CRITICAL_THRESHOLD_RIGHT_TURN = 8500.0;
        const float WALL_DISTANCE = 7000.0;

        std::cout << "begin maze solver" << std::endl;
        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // declare instances for the wall-follower behavior and state
        BehaviorFollowWall followWall( diffBot, odometer, 50, WALL_DISTANCE );
        BehaviorState stateFollowWall( "Follow Wall", followWall );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        // BehaviorBrake brake9(diffBot);
        BehaviorBrake emergencyStop( diffBot );
        BehaviorState EMERGENCY_STOP( "EMERGENCY STOP", emergencyStop );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // declare instances for the turn-right behavior and state
        float turnRadius = WALL_DISTANCE - 2000;
        BehaviorCircleTurn turnRight( diffBot, odometer, turnRadius );
        BehaviorState stateTurnRight( "Turn Right", turnRight );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // declare instances for the avoid-forward-obstacle behavior and state
        BehaviorAvoidForwardObstacle  avoidForwardObstacle( diffBot, odometer );
        BehaviorState stateStartAvoidForwardObstacle( "Start Avoid Forward Obstacle", avoidForwardObstacle );
        BehaviorState stateFinishAvoidForwardObstacle( "Finish Avoid Forward Obstacle", avoidForwardObstacle );
        BehaviorState stateEvasiveAction( "EVASIVE ACTION!", avoidForwardObstacle );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        ModeTransition* transitToStartAvoidance =
            new ModeTransition( &stateStartAvoidForwardObstacle, [&](){
                float dForward   = diffBot.getForwardPoint().getOriginDistance();
                if( dForward < FORWARD_FAR_THRESHOLD ) { std::cout << "### getForwardPoint distance is " << dForward << std::endl; }
                return ( dForward < FORWARD_FAR_THRESHOLD );
            });

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        BehaviorDriveStraightDifferentialPID  driveStraight( diffBot, odometer, 50, 2000.0 );
        BehaviorState stateDriveStraight( "Drive Straight", driveStraight );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // set up the transitions for the follow-wall state
        stateFollowWall.pushTransition( transitToStartAvoidance );
        stateFollowWall.pushTransition( new ModeTransition( &stateDriveStraight, [&](){
            float d = diffBot.getEopdStarboardPoint().getOriginDistance();
            if( d > WALL_DISTANCE+2500 ) { std::cout << "### getEopdPoint distance is " << d << std::endl; }
            return ( d > WALL_DISTANCE+2500 );
        }));
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        stateDriveStraight.pushTransition( transitToStartAvoidance );
        stateDriveStraight.pushTransition(
            new ModeTransition( &stateTurnRight, [&](){
                bool isDone = driveStraight.isDone();
                if( isDone ) { std::cout << "### stateDriveStraight is done" << std::endl; }
                return isDone;
            })
        );

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // set up the transitions for the turn-right state
        stateTurnRight.pushTransition(
            new ModeTransition( &stateEvasiveAction, [&](){
                float dStarboard = diffBot.getStarboardPoint().getOriginDistance();
                if( dStarboard < STARBOARD_CRITICAL_THRESHOLD_RIGHT_TURN ) { std::cout << "### getStarboardPoint distance is " << dStarboard << std::endl; }
                return ( dStarboard < STARBOARD_CRITICAL_THRESHOLD_RIGHT_TURN );
            })
        );
        stateTurnRight.pushTransition( transitToStartAvoidance );
        stateTurnRight.pushTransition(
            new ModeTransition( &stateFollowWall, [&](){
                float d = diffBot.getEopdStarboardPoint().getOriginDistance();
                if( d < WALL_DISTANCE+1000 ) { std::cout << "### getEopdPoint distance is " << d << std::endl; }
                return ( d < WALL_DISTANCE+1000 );
            })
        );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // set up the transitions for the avoid-forward-obstance state
        stateStartAvoidForwardObstacle.pushTransition(
            new ModeTransition( &stateFinishAvoidForwardObstacle, [&](){
                float d = diffBot.getStarboardPoint().getOriginDistance();
                std::cout << "(start) avoid forward obstacle, starboard point distance = " << d << std::endl;
                return d < STARBOARD_NEAR_THRESHOLD_LEFT_TURN;
            })
        );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // set up the transitions for the evasive-action state
        stateEvasiveAction.pushTransition(
            new ModeTransition( &stateFinishAvoidForwardObstacle, [&](){
                float d = diffBot.getStarboardPoint().getOriginDistance();
                std::cout << "stateEvasiveAction, starboard point distance = " << d << std::endl;
                return d > STARBOARD_CRITICAL_THRESHOLD_RIGHT_TURN + 500.0;
            })
        );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        //___________________________________________________________________
        /////////////////////////////////////////////////////////////////////
        // set up the transitions for the avoid-forward-obstance state
        stateFinishAvoidForwardObstacle.pushTransition(
            new ModeTransition( &stateFollowWall, [&](){
                float d = diffBot.getStarboardPoint().getOriginDistance();
                std::cout << "(finish) avoid forward obstacle, starboard point distance = " << d << std::endl;
                return d > STARBOARD_FAR_THRESHOLD_LEFT_TURN;
            })
        );
        stateFinishAvoidForwardObstacle.pushTransition( transitToStartAvoidance );
        //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


        /////////////////////////////////////////////////////////////////////
        // initialize the currentState and start the behavior
        BehaviorState* currentState = &stateFollowWall;
        Behavior* currentBehavior = currentState->getBehavior();
        currentBehavior->start();

        try
        {
            int approxSeconds = 75;
            int tickPeriodMS  = 20;
            int tickPeriodUS  = tickPeriodMS * 1000;
            // std::cout << "tickPeriodUS is " << tickPeriodUS << "us" << std::endl;
            int numIterations = approxSeconds * 1000 / tickPeriodMS;
            int smoothSleepTimeUS = 0;
            for( int i=0; i<numIterations && !ev3dev::button::back.pressed(); ++i )
            {
                unsigned frameStartTime = usNow();
                // get sensor data and compute new robot state values
                diffBot.captureFrame();
                odometer.update();

                // std::cout << frameStartTime << " -- check for new behavior state trigger" << std::endl;
                // check to see if a new behavior state is being triggered
                BehaviorState* newState = currentState->checkForNewBehaviorState();
                if( newState )
                {
                    std::cout << "change behavior to " << newState->getName() << std::endl;
                    // currentBehavior->stop();
                    currentState = newState;
                    currentBehavior = currentState->getBehavior();
                    currentBehavior->start();
                }
                // update the robot using the current behavior
                currentBehavior->update();

                // calculate how long to sleep 
                unsigned elapsedTime = usNow() - frameStartTime;
                if( tickPeriodUS > elapsedTime )
                {
                    // std::cout << "sleep for " << tickPeriodUS - elapsedTime << "us" << std::endl;
                    smoothSleepTimeUS = smoothFilter( tickPeriodUS - elapsedTime, smoothSleepTimeUS, 0.95 );
                    usleep( tickPeriodUS - elapsedTime );
                }
            }
            std::cout<< "smoothSleepTimeUS="<<smoothSleepTimeUS << std::endl;
        }
        catch(const std::system_error& e)
        {
            std::cout << "MazeSolver caught a system_error with code " << e.code() 
                      << " meaning " << e.what() << '\n';
        }

        BehaviorBrake brake(diffBot);
        brake.start();

    };
private:
    DifferentialBot diffBot;
    Odometer odometer;
};



int main( int argc, char* argv[] )
{
    try {
        // eopdFilterTest();

        // BotManager botManager;
        // botManager.run();

        // DifferentialBot diffBot;
        // Odometer odometer(diffBot);
        // BehaviorCircleTurn turnRight( diffBot, odometer, 9000 );
        // BehaviorTest( turnRight, diffBot, odometer );

        MazeSolver mazeSolver;
        mazeSolver.run();
    } catch(const std::system_error& e) {
        std::cout << "Caught system_error with code " << e.code() 
                  << " meaning " << e.what() << '\n';
    }
}
