#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>

namespace mc_control {
  
  struct MC_CONTROL_DLLAPI MCStandupController : public MCController {
    
  public:
    
    MCStandupController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);
    
    virtual std::ostream & log_header(std::ostream & os);
    virtual std::ostream & log_data(std::ostream & os);
    
    virtual bool run() override;
    virtual void reset(const ControllerResetData & reset_data) override;
    
    std::shared_ptr<mc_tasks::OrientationTask> orBodyTask;   
    std::shared_ptr<mc_tasks::CoMTask> comTask;

    std::vector<double> encoder_prev;
    std::vector<double> alpha_prev;

    //std::vector< std::vector<double> > q_calc;
    //std::vector< std::vector<double> > alpha_calc;
    //std::vector< std::vector<double> > alphaD_calc;

    bool first = true;
    int nrIter = 0;
  };
  
  SIMPLE_CONTROLLER_CONSTRUCTOR("Standup", mc_control::MCStandupController)
}
