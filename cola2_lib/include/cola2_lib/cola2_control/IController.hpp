#include <iostream>
#include <math.h>
#include <map>

#ifndef __CONTROLLER_INTERFACE
#define __CONTROLLER_INTERFACE

#define __PI__ 3.141592653

class IController
{
public:
    IController( std::string name ):
        _name( name )
    {}

    ~IController(){}

    virtual void
    reset()
    {}

    virtual double
    compute(double time_in_sec, double setpoint, double feedback)
    {
        return 0.0;
    }

    virtual bool
    setParameters( std::map< std::string, double > params )
    {
        return false;
    }

protected:
    std::string _name;
};

double
_saturateValue( double value, double limit )
{
    if( value > limit ) value = limit;
    if( value < -limit ) value = -limit; 
    return value;
}

double
_normalizeAngle( const double angle )
{
    return (angle + ( 2.0 * __PI__ * floor( ( __PI__ - angle ) / ( 2.0 * __PI__ ) ) ) );
}

#endif // __CONTROLLER_INTERFACE
