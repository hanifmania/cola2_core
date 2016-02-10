#include "cola2_lib/cola2_control/NDofController.h"

NDofController::NDofController(const unsigned int n_dof) :
  _n_dof(n_dof)
{}

void NDofController::addController(IController *controller)
{
  assert(_controllers.size() < _n_dof);
  _controllers.push_back(controller);
}

void NDofController::setControllerParams(std::vector< std::map< std::string, double > > params)
{
  assert(params.size() == _n_dof);
  assert(_controllers.size() == _n_dof);
  for (unsigned int i = 0; i < _n_dof; i++)
  {
    _controllers.at(i)->setParameters(params.at(i));
  }
}

void NDofController::reset()
{
  for (unsigned int i = 0; i < _n_dof; i++)
  {
    _controllers.at(i)->reset();
  }
}

std::vector<double> NDofController::compute(double time_in_sec, Request req, std::vector<double> feedback)
{
  assert(_controllers.size() == _n_dof);
  std::vector< double > ret;
  std::vector< double > setpoint = req.getValues();
  std::vector< bool > disable_axis = req.getDisabledAxis();

  for (unsigned int i = 0; i < _n_dof; i++)
  {
    if (!disable_axis.at(i))
    {
      ret.push_back(_controllers.at(i)->compute(time_in_sec, setpoint.at(i), feedback.at(i)));
    }
    else
    {
      _controllers.at(i)->compute(time_in_sec, setpoint.at(i), feedback.at(i));
      _controllers.at(i)->reset();
      ret.push_back(0.0);
    }
  }
  return ret;
}
