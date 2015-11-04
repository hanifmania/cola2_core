#ifndef __POLY_CLASS__
#define __POLY_CLASS__

#include "IController.hpp"
#include <sstream>
#include <string>

class Poly: public IController {
public:
    Poly(std::string name):
        IController( name ),
        _setpoint_coefs()
        {}

    ~Poly() {}

    void reset()
    {}

    double
    compute(double time_in_sec, double setpoint, double feedback)
    {
        double tau = 0.0;
        bool negative_setpoint = false;
        if( setpoint < 0 ) negative_setpoint = true;

        for( unsigned int i = 0; i < _setpoint_coefs.size(); i++ ){
            tau = tau + pow( double( fabs(setpoint) ), double( i ) ) * _setpoint_coefs.at(i);
        }
        if(negative_setpoint) tau = -1.0 * tau;

        // Return non-saturated tau
        return tau;
    }


    bool
    setParameters( std::map< std::string, double > params )
    {
        std::cout << "Set params for " << _name << ": " << int(params["1"]) << "\n";
        _setpoint_coefs.clear();
        try {
            unsigned int n_dof = int(params["n_dof"]);
            // std::cout << "poly n_dof: " << n_dof << "\n";
            for( unsigned int i = 0; i < n_dof; i++ ){
                std::ostringstream s;
                s << i;
                const std::string i_as_string( s.str() );
                _setpoint_coefs.push_back( double( params[i_as_string] ) );
                // std::cout << "add param: " << double( params[i_as_string] ) << "\n";
            }
        }
        catch (...) {
            return false;
        }
        // std::cout << "Poly initialized!\n";
        return true;
    }


private:
    std::vector< double > _setpoint_coefs;
};

#endif //__POLY_CLASS__
