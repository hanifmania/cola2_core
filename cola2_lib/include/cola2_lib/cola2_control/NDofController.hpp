#include "IController.hpp"
#include "Request.hpp"
#include <assert.h>
#include <vector>

#ifndef __NDOFCONTROLLER_CLASS__
#define __NDOFCONTROLLER_CLASS__

class NDofController {
public:
    NDofController( const unsigned int n_dof = 6 ) :
        _n_dof( n_dof )
    {}


    ~NDofController()
    {}


    void
    addController( IController *controller )
    {
        assert( _controllers.size() < _n_dof );
        _controllers.push_back( controller );
    }


    void
    setControllerParams( std::vector< std::map< std::string, double > > params )
    {
        assert( params.size() == _n_dof );
        assert( _controllers.size() == _n_dof );
        for( unsigned int i = 0; i < _n_dof; i++ ) {
            _controllers.at(i)->setParameters( params.at(i) );
        }
    }


    void
    reset(){
        for( unsigned int i = 0; i < _n_dof; i++ ) {
            _controllers.at( i )->reset();
        }
    }


    std::vector< double >
    compute( double time_in_sec,
             Request req,
             std::vector< double > feedback )
    {
        assert( _controllers.size() == _n_dof );
        std::vector< double > ret;
        std::vector< double > setpoint = req.getValues();
        std::vector< bool > disable_axis = req.getDisabledAxis();

        for( unsigned int i = 0; i < _n_dof; i++ ) {
            if( !disable_axis.at( i ) ){
                ret.push_back( _controllers.at( i )->compute( time_in_sec,
                                                              setpoint.at(i),
                                                              feedback.at(i) ) );
            }
            else {
                _controllers.at( i )->compute( time_in_sec, setpoint.at(i), feedback.at(i) );
                _controllers.at( i )->reset();
                ret.push_back( 0.0 );
            }
        }
        return ret;
    }


private:
    std::vector< IController* > _controllers;
    unsigned int _n_dof;
};

#endif //__NDOFCONTROLLER_CLASS__
