#include <mc_rtc/logging.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include "mc_standup_controller.h"

namespace mc_control {
  
  MCStandupController::MCStandupController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
    : MCController(robot_module, dt) {
    
    contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Acceleration);
    
    postureTask->stiffness(300000.0); // 300000
    postureTask->weight(500.0);
    
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    solver().addTask(postureTask.get());
    
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 500.0, 100.0);
    auto mbc = robot().mbc();
    mbc.q = postureTask->posture();
    auto comT = rbd::computeCoM(robot().mb(), mbc);
    comTask->com(comT);
    solver().addTask(comTask);

    orBodyTask = std::make_shared<mc_tasks::OrientationTask>("Body", robots(), 0, 50.0, 100.0);
    orBodyTask->orientation(Eigen::Matrix3d::Identity());
    solver().addTask(orBodyTask);
    
    //solver().setContacts({});
    
    solver().setContacts({{robots(), 0, 1, "LeftFoot",  "AllGround"},
			  {robots(), 0, 1, "RightFoot", "AllGround"}});

    LOG_SUCCESS("MCStandupController init done" << this);
  }
  
  bool MCStandupController::run() {

    nrIter++;
    //q_calc = robot().mbc().q;
    //alpha_calc = robot().mbc().alpha;
    //alphaD_calc = robot().mbc().alphaD;

    if (first) {
      encoder_prev = robot().encoderValues();
      alpha_prev.resize(robot().mb().nrJoints(), 0.0);
      first = false;
    }
    else
      encoder_prev.resize(robot().encoderValues().size(), 0.0);
    
    const std::vector<double> & encoder = robot().encoderValues();
    
    for (size_t i = 0; i < robot().refJointOrder().size(); ++i) {
      
      const auto & jn = robot().refJointOrder()[i];
      if (robot().hasJoint(jn)) {
        size_t j = robot().jointIndexByName(jn);
	robot().mbc().q[j][0] = encoder[i];
        robot().mbc().alpha[j][0] = (encoder[i] - encoder_prev[i]) / timeStep;
        robot().mbc().alphaD[j][0] = (robot().mbc().alpha[j][0] - alpha_prev[j]) / timeStep;
        alpha_prev[j] = robot().mbc().alpha[j][0];
      }
    }

    encoder_prev = encoder;

    rbd::forwardKinematics(robot().mb(), robot().mbc());
    rbd::forwardVelocity(robot().mb(), robot().mbc());
    
    bool ret = MCController::run();
    //ret = ret && MCController::run();
    return ret;
  }

  void MCStandupController::reset(const ControllerResetData & reset_data) {
    MCController::reset(reset_data);
  }

  std::ostream & MCStandupController::log_header(std::ostream & os) {
    
    //for (size_t j = 0; j < (size_t) robot().mb().nrJoints(); ++j)
    //  os << ";qRef" << j;

    //for (size_t j = 0; j < (size_t) robot().mb().nrJoints(); ++j)
    //  os << ";qCalc" << j;
    
    return os;
  }

  std::ostream & MCStandupController::log_data(std::ostream & os) {
    
    const std::vector< std::vector<double> > q_ref = postureTask->posture();
    
    //for (size_t j = 0; j < q_ref.size(); ++j)
    //  os << ";" << q_ref[j][0];

    //for (size_t j = 0; j < q_calc.size(); ++j)
    //  os << ";" << q_calc[j][0];
    
    return os;
  }
}
