#include "cola2_lib/cola2_navigation/EkfBase.h"

EkfBase::EkfBase(const unsigned int state_vector_size, const Eigen::VectorXd q_var):
  _state_vector_size(state_vector_size),
  _last_prediction(0.0),
  _is_ekf_init(false),
  _filter_updates(0)
{
  _x.resize(_state_vector_size, 1);
  _x = Eigen::MatrixXd::Zero(_state_vector_size, 1);
  _P.resize(_state_vector_size, _state_vector_size);
  _P = Eigen::MatrixXd::Identity(_state_vector_size, _state_vector_size);
  _Q.resize(q_var.size(), q_var.size());
  _Q = Eigen::ArrayXXd::Zero(q_var.size(), q_var.size());
  for (unsigned int i = 0; i < q_var.size(); i++)
  {
    _Q(i, i) = q_var(i);
  }
  std::cout << "_x size = " << _x.size() << "\n";
  std::cout << "_P size = " << _P.size() << "\n";
}

void EkfBase::initEkf(const Eigen::VectorXd state, const Eigen::VectorXd p_var)
{
  assert(state.size() == _state_vector_size);
  assert(p_var.size() == _state_vector_size);

  // Init state vector
  _x = Eigen::MatrixXd::Zero(_state_vector_size, 1);
  _P = Eigen::MatrixXd::Identity(_state_vector_size, _state_vector_size);

  for (unsigned int i = 0; i < _state_vector_size; i++)
  {
    _x(i) = state(i);
    _P(i, i) = p_var(i);
  }

  showStateVector();

  _filter_updates = 0;
  _is_ekf_init = true;
}

bool EkfBase::makePrediction(const double now, Eigen::VectorXd u)
{
  if (_is_ekf_init)
  {
    double period = now - _last_prediction;

    // std::cout << "u:\n" << u << "\n";
    // std::cout << "period: " << period << "\n";

    if (period > 0.0 && period <= 1.0)
    {
      Eigen::MatrixXd A = computeA(period, u);
      // std::cout << "\nA:\n " << A << "\n";

      Eigen::MatrixXd W = computeW(period, u);
      // std::cout << "\nW:\n " << W << "\n";

      _x_ = f(_x, period, u);
      // std::cout << "\n_x_:\n " << _x_ << "\n";
      // std::cout << "_P\n" << _P << "\n";
      // std::cout << "_Q\n" << _Q << "\n";

      _P_ = A * _P * A.transpose() + W * _Q * W.transpose();
      // std::cout << "\n_P_:\n " << _P_ << "\n";

      _last_prediction = now;
      return true;
    }
    else if (period > -0.1 && period <= 0.0)
    {
      _x_ = _x;
      _P_ = _P;
      return true;
    }
    else
    {
      std::cerr << "makePrediction invalid period " << period << "\n";
      _last_prediction = now;  // Update time, otherwise everything will fail!
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool EkfBase::applyUpdate(const Eigen::VectorXd z,
                          const Eigen::MatrixXd r,
                          const Eigen::MatrixXd h,
                          const Eigen::MatrixXd v,
                          const double mahalanobis_distance_threshold)
{
  double distance = mahalanobisDistance(z, r, h);
  // std::cout << "mahalanobisDistance distance: " << distance << "\n";
  if (_filter_updates < 100 || distance < mahalanobis_distance_threshold)
  {
    Eigen::VectorXd innovation = z - h*_x_;
    normalizeInnovation(innovation);
    // std::cout << "Normalized innovation:\n" << innovation << "\n";
    // Compute updated state vector
    Eigen::MatrixXd S = h * _P_ * h.transpose() + v * r * v.transpose();
    // std::cout << "S:\n" << S << "\n";

    Eigen::MatrixXd K = _P_* h.transpose()* S.inverse();
    // std::cout << "K:\n" << K << "\n";

    _x = _x_ + K*innovation;
    normalizeState();

    // Compute updated covariance matrix
    unsigned int I_size = _x.size();
    _P = (Eigen::MatrixXd::Identity(I_size, I_size) - K * h) * _P_;

    // Alternative method to perform covariance update
    // Eigen::MatrixXd T = Eigen::MatrixXd::Identity(I_size, I_size) - K * h;
    // _P = T * _P_ * T.transpose() + K * r * K.transpose();

    // Check integrity
    checkIntegrity();

    _filter_updates++;
  }
  else
  {
    return false;
  }
  return true;
}

void EkfBase::showStateVector()
{
  std::cout << "state Vector:\n" << _x << "\n\n";
  std::cout << "P:\n" << _P << "\n\n";
}

Eigen::VectorXd EkfBase::getStateVector()
{
  return _x;
}

Eigen::MatrixXd EkfBase::getCovarianceMatrix()
{
  return _P;
}

void EkfBase::setLastPredictionTime(const double time)
{
  _last_prediction = time;
}

void EkfBase::setTransformation(const std::string sensor_id,
                                const Eigen::Quaterniond rotation_from_origin,
                                const Eigen::Vector3d translation_from_origin)
{
  std::pair< Eigen::Quaterniond, Eigen::Vector3d > value(rotation_from_origin, translation_from_origin);
  std::pair< std::string, std::pair< Eigen::Quaterniond, Eigen::Vector3d > > element(sensor_id, value);
  _transformations.insert(element);
  std::cout << "Set transformation for " << sensor_id << "\n";
  std::cout << "trans: \n" << translation_from_origin << "\n";
  std::cout << "rotation: " << rotation_from_origin.x() << ", " << rotation_from_origin.y() << ", "
            << rotation_from_origin.z()  << ", " << rotation_from_origin.w() << "\n";
}

// *****************************************
//          Utility functions
// *****************************************
double EkfBase::mahalanobisDistance(const Eigen::VectorXd z,
                                    const Eigen::MatrixXd r,
                                    const Eigen::MatrixXd h)
{
  Eigen::VectorXd v = z - h*_x;
  Eigen::MatrixXd S = h*_P*h.transpose() + r;
  Eigen::VectorXd d = v.transpose() * S.inverse() * v;
  return sqrt(d(0, 0));
}

void EkfBase::checkIntegrity()
{
  // NaN check
  if (isnan(_x(0)))
  {
    std::cout << "\033[1;31m" << "NaNs detected!!!" << "\033[0m\n";
  }

  // P matrix
  for (unsigned int i = 0; i < _x.size(); i++)
  {
    if (_P(i, i) < 0.0)
    {
      _P(i, i) = 0.01;
      std::cout << "Negative values in P(" << i << "," << i << ")\n";
    }
  }
}

bool EkfBase::isValidCandidate(const std::vector< Eigen::VectorXd >& candidates)
{
  std::cout << "Is the candidate good enough to enter in the filter?\n";
  if (candidates.size() >= 4)
  {
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(4, candidates.at(0).size());
    unsigned int j = 0;
    for (unsigned int i = candidates.size()-4; i < candidates.size(); i++)
    {
      // std::cout << "i: " << i << "\n";
      // std::cout << "mat block:\n" << mat.block(i, 0, 1, 6) << "\n";
      // std::cout << "candidate:\n" << candidates.at(i).transpose() << "\n";
      mat.block(j, 0, 1, candidates.at(0).size()) = candidates.at(i).transpose();
      j++;
    }
    std::cout << "last candidates are:\n" << mat << "\n";

    // TODO: Check that these equations work.
    Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
    Eigen::MatrixXd cov = (centered.adjoint() * centered) / static_cast<double>(mat.rows());
    std::cout << "Covariance: \n" << cov.diagonal() << "\n";
    for (unsigned int i = 0; i < candidates.at(0).size() ; i++)
    {
      if (cov(i, i) > MAX_COV) return false;
    }
    std::cout << "The candidate is valid!\n";
    return true;
  }
  return false;
}

void EkfBase::updatePrediction()
{
  _x = _x_;
  _P = _P_;
  // Check covariabce matrix integrity
  checkIntegrity();
}
