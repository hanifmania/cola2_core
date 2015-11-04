#include <assert.h>
#include <iostream>
#include <vector>

#ifndef __REQUEST_CLASS__
#define __REQUEST_CLASS__

class Request {
public:
    Request( std::string requester = "", const double stamp = 0.0,
             const unsigned int priority = 0, const unsigned int n_dof = 6 ) :
        _requester( requester ),
        _stamp( stamp ),
        _priority( priority ),
        _n_dof( n_dof )
    {
        for( unsigned int i = 0; i < _n_dof; i++ ) {
            _disabled_axis.push_back( true );
            _values.push_back( 0.0 );
        }
    }

    Request( const Request& other ):
        _requester( other._requester ),
        _stamp( other._stamp ),
        _disabled_axis( other._disabled_axis ),
        _values( other._values ),
        _priority( other._priority ),
        _n_dof( other._n_dof )
    {}

    ~Request(){}

    void
    combineRequest( const Request& req,
                         std::string mode )
    {
        assert( req._n_dof == _n_dof );
        assert( req._priority <= _priority );
        if( req._priority < _priority ) {
            combineLowerPriorityRequest( req );
        }
        else if( req._priority == _priority ) {
            combineSamePriorityRequest( req, mode );
        }
    }

    void
    combineSamePriorityRequest( const Request& req,
                                std::string mode )
    {
        // If same priority but can be combined, combine them
        if( matchDisabledAxis( req._disabled_axis ) ) {
            combineRequest( req );
        }
        // Otherwise, if pose show error
        else if( mode == "pose" ){
            std::cerr << "Impossible to combine poses with same priority!\n";
        }
        // If twist or wrench: If only one has value put this one otherwise add them.
        else if ( mode == "twist" || mode == "wrench" ){
            for( unsigned int i = 0; i < _n_dof; i++ ) {
                if( !req._disabled_axis.at( i ) ) {
                    if( _disabled_axis.at( i ) ) {
                        _values.at( i ) = req._values.at( i );
                        _disabled_axis.at( i ) = false;
                    }
                    else {
                        _values.at( i ) = req._values.at( i ) + _values.at( i );
                    }
                }
            }
        }
    }

    void
    combineLowerPriorityRequest( const Request& lower )
    {
        assert( lower._priority < _priority );

        // If it is possible to combine
        if( matchDisabledAxis( lower._disabled_axis ) ) {
            combineRequest( lower );
        }
    }


    void
    combineRequest( const Request& req )
    {
        for( unsigned int i = 0; i < _n_dof; i++ ) {
            if( !req._disabled_axis.at( i ) ) {
                _disabled_axis.at( i ) = false;
                _values.at( i ) = req._values.at( i );
            }
        }
    }


    bool
    matchDisabledAxis( std::vector< bool > axis )
    {
        assert( axis.size() == _disabled_axis.size() );
        bool is_matched = true;
        unsigned int i = 0;
        while( is_matched && i < axis.size() ) {
            if( !axis.at( i ) ) {
                if( !_disabled_axis.at( i ) ) {
                    is_matched = false;
                }
            }
            i++;
        }
        // std::cout << "matchDisabledAxis " << _requester << ": " << is_matched << "\n";
        return is_matched;
    }


    void
    setDisabledAxis( std::vector< bool > values )
    {
        assert( values.size() == _n_dof );
        std::copy( values.begin(), values.end(), _disabled_axis.begin() );
    }

    std::vector< bool >
    getDisabledAxis() const
    {
        return _disabled_axis;
    }

    void
    setValues( std::vector< double > values )
    {
        assert( values.size() == _n_dof );
        std::copy( values.begin(), values.end(), _values.begin() );
    }


    std::vector< double >
    getValues() const
    {
        return _values;
    }


    void
    setRequester(std::string name)
    {
        _requester = name;
    }


    std::string
    getRequester() const
    {
        return _requester;
    }


    double
    getStamp()
    {
        return _stamp;
    }


    unsigned int
    getPriority() const
    {
        return _priority;
    }


    void
    setPriority( const unsigned int priority )
    {
        _priority = priority;
    }


    /*
     * If at least one axis is not disabled return false, otherwise true.
     */
    bool
    isAllDisabled() const
    {
        bool disabled = true;
        for( std::vector< bool >::const_iterator it = _disabled_axis.begin();
             it != _disabled_axis.end(); it++ )
        {
            if( !*it ) disabled = false;
        }

        return disabled;
    }


    inline bool
    operator< ( const Request& rhs ) const
    {
        return ( _priority < rhs._priority );
    }

    inline bool
    operator== ( const Request& rhs ) const
    {
        return ( _requester == rhs._requester );
    }

    friend std::ostream &operator<<(std::ostream &out, Request r)
    {
        out << "requester: " << r._requester << "\n";
        out << "stamp: " << r._stamp << "\n";
        out << "priority: " << r._priority << "\n";
        out << "[ ";
        for( unsigned int i = 0; i < r._n_dof; i++ ) {
            if  (r._disabled_axis.at(i) )
                out << "true\t";
            else
                out << "false\t";
        }
        out << "]\n";
        out << "[ ";
        for( unsigned int i = 0; i < r._n_dof; i++ ) {
            out << r._values.at(i) << "\t";
        }
        out << "]\n";

        return out;
    }

private:
    std::string _requester;
    double _stamp;
    std::vector< bool > _disabled_axis;
    std::vector< double > _values;
    unsigned int _priority;
    unsigned int _n_dof;
};

#endif //__REQUEST_CLASS__
