#ifndef __AUV_CONTROLLER_INTERFACE__
#define __AUV_CONTROLLER_INTERFACE__

#include <Eigen/Dense>
#include "Merge.hpp"

class IAUVController
{
public:
    IAUVController( double period, int n_dof, int n_thrusters, int n_fins = 0 ):
        _is_pose_controller_enable( true ),
        _is_velocity_controller_enable( true ),
        _is_thruster_allocator_enable( true ),
		_is_fin_allocator_enable( true ),
        _n_dof( n_dof ),
        _pose_merge( "pose_merge", "pose", period * 2 ),
        _pose_feedback( n_dof, 0.0 ),
        _max_velocity( n_dof, 0.2 ),
        _twist_merge( "twist_merge", "twist", period * 2 ),
        _twist_feedback( n_dof, 0.0 ),
        _max_wrench( n_dof, 100.0 ),
        _wrench_merge( "wrench_merge", "wrench", period * 2 ),
        _thruster_setpoints( n_thrusters ),
        _fin_setpoints( n_fins )
    { }


    ~IAUVController() {}


    void
    updatePoseRequest( Request req )
    {
        _mtx.lock();
        _pose_merge.addRequest( req );
        _mtx.unlock();
    }


    void 
    updatePoseFeedback( std::vector< double > feedback )
    {
        _mtx.lock();
        _pose_feedback = feedback;
        _mtx.unlock();
    }


    void
    updateTwistRequest( Request req )
    {
        _mtx.lock();
        _twist_merge.addRequest( req );
        _mtx.unlock();
    }


    void 
    updateTwistFeedback( std::vector< double > feedback )
    {
        _mtx.lock();
        _twist_feedback = feedback;
        _mtx.unlock();
    }


    void
    updateWrenchRequest( Request req )
    {
        _mtx.lock();
        _wrench_merge.addRequest( req );
        _mtx.unlock();
    }


    void
    addPolyParamToVector( std::vector< double > values,
                          std::vector< std::map< std::string, double > >& params_vector )
    {
        assert( values.size() == 3 );
        std::map< std::string, double > param;
        param.insert( std::pair< std::string, double > ( "n_dof", 3.0 ) );
        param.insert( std::pair< std::string, double > ( "0", values.at(0) ) );
        param.insert( std::pair< std::string, double > ( "1", values.at(1) ) );
        param.insert( std::pair< std::string, double > ( "2", values.at(2) ) );
        params_vector.push_back( param );
    }


    void
    addPIDParamToVector( std::vector< std::string > keys,
                      std::vector< double > values,
                      std::vector< std::map< std::string, double > >& params_vector )
    {
        assert( keys.size() == values.size() );
        std::map< std::string, double > param;
        for(unsigned int i = 0; i < keys.size(); i++ ) {
            param.insert( std::pair< std::string, double > ( keys.at( i ), values.at( i ) ) );
        }
        // Add derivative_term_from_feedback
        param.insert( std::pair< std::string, double > ( "derivative_term_from_feedback", 1.0 ) );
        params_vector.push_back( param );
    }


    void
    setMaxWrench( std::vector<double> max_wrench )
    {
        assert( max_wrench.size() == _max_wrench.size() );
        std::copy( max_wrench.begin(), max_wrench.end(), _max_wrench.begin() );
    }


    void
    setMaxVelocity( std::vector<double> max_velocity )
    {
        assert( max_velocity.size() == _max_velocity.size() );
        std::copy( max_velocity.begin(), max_velocity.end(), _max_velocity.begin() );
    }


    Request
    getMergedPose() const
    {
        return _merged_pose;
    }


    Request
    getMergedTwist() const
    {
        return _merged_twist;
    }


    Request
    getMergedWrench() const
    {
        return _merged_wrench;
    }


    void
    setMergedWrench( const Request wrench )
    {
        _merged_wrench = wrench;
    }


    Eigen::VectorXd
    getThrusterSetpoints() const
    {
        return _thruster_setpoints;
    }

    Eigen::VectorXd
    getFinSetpoints() const
    {
        return _fin_setpoints;
    }

    virtual void
    iteration( double current_time ) = 0;

    virtual void
    reset( ) = 0;

    virtual void
    computeThrusterAllocator() = 0;

    virtual unsigned int
    getNumberofThrusters() const = 0;

    void
    setPoseController( const bool is_enabled )
    {
        _is_pose_controller_enable = is_enabled;
    }


    void
    setVelocityController( const bool is_enabled )
    {
        _is_velocity_controller_enable = is_enabled;
    }


    void
    setThrusterAllocator( const bool is_enabled )
    {
        _is_thruster_allocator_enable = is_enabled;
    }


    bool
    isThrusterAllocatorEnable() const
    {
        return _is_thruster_allocator_enable;
    }
    
    
    void
    setFinAllocator( const bool is_enabled )
    {
        _is_fin_allocator_enable = is_enabled;
    }


    bool
    isFinAllocatorEnable() const
    {
        return _is_fin_allocator_enable;
    }


    // Are controllers enabled
    bool _is_pose_controller_enable;
    bool _is_velocity_controller_enable;
    bool _is_thruster_allocator_enable;
	bool _is_fin_allocator_enable;

protected:
    // #DoF
    unsigned int _n_dof;

    // Pose
    Merge _pose_merge;
    std::vector< double > _pose_feedback;
    std::vector< double > _max_velocity;

    // Twist
    Merge _twist_merge;
    std::vector< double > _twist_feedback;
    std::vector< double > _max_wrench;

    // Wrench
    Merge _wrench_merge;

    // Mutex
    std::mutex _mtx;

    // Intermediate requests
    Request _merged_pose;
    Request _merged_twist;
    Request _merged_wrench;
    Eigen::VectorXd _thruster_setpoints;
    Eigen::VectorXd _fin_setpoints;

};

#endif //__AUV_CONTROLLER_INTERFACE__
