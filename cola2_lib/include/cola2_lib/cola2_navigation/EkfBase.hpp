#ifndef __EKF_BASE__
#define __EKF_BASE__

// Don't panic!!! This file is for the COLA2 project.
// However, as all the included files that it is going to need
// are currently developed in this other project, I've started
// to work with it here.

// #include "../../include/nav_helper/transformations.h"
// #include "../../include/nav_helper/nav_utils.h"
#include <map>
#include <iostream>
#include <utility>
#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <string>

#define MAX_COV 0.0025

class EkfBase
{
public:
    EkfBase( const unsigned int state_vector_size,
             const Eigen::VectorXd q_var ):
                _state_vector_size(state_vector_size),
                _last_prediction( 0.0 ),
                _is_ekf_init( false ),           
                _filter_updates( 0 )
    {
        _x.resize( _state_vector_size, 1 );
        _x = Eigen::MatrixXd::Zero( _state_vector_size, 1 );
        _P.resize( _state_vector_size, _state_vector_size );
        _P = Eigen::MatrixXd::Identity( _state_vector_size, _state_vector_size );
        _Q.resize( q_var.size(), q_var.size() );
        _Q = Eigen::ArrayXXd::Zero( q_var.size(), q_var.size() );
        for( unsigned int i = 0; i < q_var.size(); i++ ) {
            _Q(i, i) = q_var(i);
        }
        std::cout << "_x size = " << _x.size() << "\n";
        std::cout << "_P size = " << _P.size() << "\n";
    }


    void
    initEkf( const Eigen::VectorXd state,
             const Eigen::VectorXd p_var )
    {
        assert( state.size() == _state_vector_size );
        assert( p_var.size() == _state_vector_size );

        // Init state vector
        _x = Eigen::MatrixXd::Zero( _state_vector_size, 1 );
        _P = Eigen::MatrixXd::Identity( _state_vector_size, _state_vector_size );

        for( unsigned int i = 0; i < _state_vector_size; i++ ) {
            _x(i) = state(i);
            _P(i, i) = p_var(i);
        }

        showStateVector();

        _filter_updates = 0;
        _is_ekf_init = true;
    }


    bool
    makePrediction( const double now,
                    Eigen::VectorXd u )
    {

        if( _is_ekf_init ) {
            double period = now - _last_prediction;

            // std::cout << "u:\n" << u << "\n";
            // std::cout << "period: " << period << "\n";

            // TODO: get mutex
            if( period > 0.0 && period <= 1.0 ) {
                Eigen::MatrixXd A = computeA( period, u );
                // std::cout << "\nA:\n " << A << "\n";

                Eigen::MatrixXd W = computeW( period, u );
                // std::cout << "\nW:\n " << W << "\n";

                _x_ = f( _x, period, u );
                // std::cout << "\n_x_:\n " << _x_ << "\n";
                // std::cout << "_P\n" << _P << "\n";
                // std::cout << "_Q\n" << _Q << "\n";

                _P_ = A * _P * A.transpose() + W * _Q * W.transpose();
                // std::cout << "\n_P_:\n " << _P_ << "\n";

                _last_prediction = now;
                return true;
            }
            else if( period > -0.1 && period <= 0.0 ) {
                _x_ = _x;
                _P_ = _P;
                return true;
            }
            else {
                std::cerr << "makePrediction invalid period " << period << "\n";
                _last_prediction = now; // Update time, otherwise everything will fail!
                return false;
            }
            // TODO: release mutex
        }
        else {
            return false;
        }
    }


    bool
    applyUpdate( const Eigen::VectorXd z,
                 const Eigen::MatrixXd r,
                 const Eigen::MatrixXd h,
                 const Eigen::MatrixXd v,
                 const double mahalanobis_distance_threshold )
    {
        double distance = mahalanobisDistance( z, r, h );
        // std::cout << "mahalanobisDistance distance: " << distance << "\n";
        if( _filter_updates < 100 || distance < mahalanobis_distance_threshold ) {
            // TODO: get mutex

            Eigen::VectorXd innovation = z - h*_x_;
            normalizeInnovation( innovation );
            // std::cout << "Normalized innovation:\n" << innovation << "\n";
            // Compute updated state vector
            Eigen::MatrixXd S = h * _P_ * h.transpose() + v * r * v.transpose();
            //std::cout << "S:\n" << S << "\n";

            Eigen::MatrixXd K = _P_* h.transpose()* S.inverse();
            //std::cout << "K:\n" << K << "\n";

            _x = _x_ + K*innovation;
            normalizeState();

            // Compute updated covariance matrix
            unsigned int I_size = _x.size();
            _P = ( Eigen::MatrixXd::Identity(I_size, I_size) - K * h ) * _P_;

            // Alternative method to perform covariance update
            // Eigen::MatrixXd T = Eigen::MatrixXd::Identity(I_size, I_size) - K * h;
            // _P = T * _P_ * T.transpose() + K * r * K.transpose();

            // Check integrity
            checkIntegrity();

            _filter_updates++;
            // TODO: release mutex           
        }
        else {
            return false;
        }
        return true;
    }

    void
    showStateVector()
    {
        std::cout << "state Vector:\n" << _x << "\n\n";
        std::cout << "P:\n" << _P << "\n\n";
    }

    Eigen::VectorXd
    getStateVector()
    {
        return _x;
    }


    Eigen::MatrixXd
    getCovarianceMatrix()
    {
        return _P;
    }

    void
    setLastPredictionTime( const double time )
    {
        _last_prediction = time;
    }


    void
    setTransformation( const std::string sensor_id,
                       const Eigen::Quaterniond rotation_from_origin,
                       const Eigen::Vector3d translation_from_origin )
    {
        std::pair< Eigen::Quaterniond, Eigen::Vector3d > value( rotation_from_origin, translation_from_origin );
        std::pair< std::string, std::pair< Eigen::Quaterniond, Eigen::Vector3d > > element( sensor_id, value );
        _transformations.insert( element );
        std::cout << "Set transformation for " << sensor_id << "\n";
        std::cout << "trans: \n" << translation_from_origin << "\n";
        std::cout << "rotation: " << rotation_from_origin.x() << ", " << rotation_from_origin.y() << ", "
                  << rotation_from_origin.z()  << ", " << rotation_from_origin.w() << "\n";
    }

protected:
    // *****************************************
    //          Utility functions
    // *****************************************


    double
    mahalanobisDistance( const Eigen::VectorXd z,
                         const Eigen::MatrixXd r,
                         const Eigen::MatrixXd h )
    {
        Eigen::VectorXd v = z - h*_x;
        Eigen::MatrixXd S = h*_P*h.transpose() + r;
        Eigen::VectorXd d = v.transpose() * S.inverse() * v;
        return sqrt( d( 0, 0 ) );
    }

    void
    checkIntegrity()
    {
        // NaN check
        if ( isnan( _x(0) ) ) {
            std::cout << "\033[1;31m" << "NaNs detected!!!" << "\033[0m\n";
        }

        // P matrix
        for (unsigned int i = 0; i < _x.size(); i++) {
            if (_P(i,i) < 0.0) {
                _P(i,i) = 0.01;
                    std::cout << "Negative values in P(" << i << "," << i << ")\n";
            }
        }
    }


    bool
    isValidCandidate( const std::vector< Eigen::VectorXd >& candidates )
    {
        std::cout << "Is the candidate good enough to enter in the filter?\n";
        if( candidates.size() >= 4 ) {
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero( 4, candidates.at(0).size() );
            unsigned int j = 0;
            for( unsigned int i = candidates.size()-4; i < candidates.size(); i++ ) {
                // std::cout << "i: " << i << "\n";
                // std::cout << "mat block:\n" << mat.block(i, 0, 1, 6) << "\n";
                // std::cout << "candidate:\n" << candidates.at(i).transpose() << "\n";
                mat.block( j, 0, 1, candidates.at(0).size() ) = candidates.at(i).transpose();
                j++;
            }
            std::cout << "last candidates are:\n" << mat << "\n";

            // TODO: Check that these equations work.
            Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
            Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(mat.rows());
            std::cout << "Covariance: \n" << cov.diagonal() << "\n";
            for( unsigned int i = 0; i < candidates.at(0).size() ; i++ ){
                if( cov(i, i) > MAX_COV ) return false;
            }
            std::cout << "The candidate is valid!\n";
            return true;
        }
        return false;
    }


    void
    updatePrediction()
    {
        // TODO: get mutex
        _x = _x_;
        _P = _P_;
        // --------- Check covariabce matrix integrity ----------
        checkIntegrity();
        // TODO: release mutex
    }


    unsigned int _state_vector_size;
    double _last_prediction;
    Eigen::MatrixXd _Q;
    bool _is_ekf_init;
    Eigen::MatrixXd _x;
    Eigen::MatrixXd _P;
    Eigen::MatrixXd _x_;
    Eigen::MatrixXd _P_;
    std::map< std::string, std::pair< Eigen::Quaterniond, Eigen::Vector3d  > > _transformations;
    unsigned int _filter_updates;

private:
    // *****************************************
    //            Methods that must be
    //      implemented in derived calsses
    // *****************************************

    virtual void
    normalizeInnovation( Eigen::VectorXd& innovation ) const = 0;

    virtual void
    normalizeState() = 0;

    virtual Eigen::MatrixXd
    computeA( const double t,
              const Eigen::VectorXd u ) const = 0;

    virtual Eigen::MatrixXd
    computeW( const double t,
              const Eigen::VectorXd u ) const = 0;

    virtual Eigen::VectorXd
    f( const Eigen::VectorXd x_1,
       const double t,
       const Eigen::VectorXd u ) const = 0;

    virtual Eigen::VectorXd
    computeU() const = 0;
};

#endif // __EKF_BASE__
