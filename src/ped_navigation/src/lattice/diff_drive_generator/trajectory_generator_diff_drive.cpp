#include "lattice/diff_drive_generator/trajectory_generator_diff_drive.h"
//#include "lattice/decision_maker.h"
TrajectoryGeneratorDiffDrive::TrajectoryGeneratorDiffDrive(void)
{
    // default
    h << 0.05, 0.05, 0.1;
    MAX_YAWRATE = 1.0;
    _lamda << 0.05, 0.05, 0.1;
}

void TrajectoryGeneratorDiffDrive::set_optimization_param(const double dkm, const double dkf, const double dsf)
{
    h << dkm, dkf, dsf;
}

void TrajectoryGeneratorDiffDrive::set_motion_param(const double max_yawrate, const double max_d_yawrate, const double max_acceleration, const double max_wheel_angular_velocity, const double wheel_radius, const double tread)
{
    MAX_YAWRATE = max_yawrate;
    model.set_param(max_yawrate, max_d_yawrate, max_acceleration, max_wheel_angular_velocity, wheel_radius, tread);
}

void TrajectoryGeneratorDiffDrive::set_verbose(bool verbose_)
{
    verbose = verbose_;
}

double TrajectoryGeneratorDiffDrive::generate_optimized_trajectory(const Eigen::Vector3d& goal, const MotionModelDiffDrive::ControlParams& init_control_param, const double dt, const double tolerance, const int max_iteration, MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory)
{
    Eigen::Vector3d cost(1e2, 1e2, 1e2);
    double last_cost = cost.norm();
    double distance_to_goal = goal.segment(0, 2).norm();

    output = init_control_param;

    int count = 0;

    Eigen::Matrix3d jacobian;

    while(1){
        // std::cout << "---" << std::endl;
        if(cost.norm() < tolerance){
            if(verbose){
                std::cout << "successfully optimized in " << count << " iteration" << std::endl;
            }
            break;
        }else if(count >= max_iteration){
            if(verbose){
                std::cout << "cannot optimize trajectory" << std::endl;
            }
            return -1;
        }
        trajectory.trajectory.clear();
        trajectory.velocities.clear();
        trajectory.angular_velocities.clear();
        model.generate_trajectory(dt, output, trajectory);
        /*
        std::cout << "size: " << trajectory.trajectory.size() << std::endl;
        if(trajectory.trajectory.size() <= 1){
            std::cout << "failed to generate trajecotry!!!" << std::endl;
            return -1;
        }
        */

        get_jacobian(dt, output, h, jacobian);
        // std::cout << "j: \n" << jacobian << std::endl;
        // std::cout << "j^-1: \n" << jacobian.inverse() << std::endl;
        cost = goal - trajectory.trajectory.back();
        // Eigen::Vector3d dp = jacobian.inverse() * cost;
        //std::cout <<"dp before"<<std::endl;
        Eigen::Vector3d dp = jacobian.lu().solve(cost);
        //std::cout <<"dp after"<<std::endl;
        if(std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) || std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2)) || fabs(dp(2)) > distance_to_goal){
            if(verbose){
                std::cout << "diverge to infinity!!!" << std::endl;
            }
            return -1;
        }
        // std::cout << "cost vector: " << cost.transpose() << std::endl;
        // std::cout << "cost: " << cost.norm() << std::endl;
        // std::cout << "dp" << dp.transpose() << std::endl;
        // std::cout << "calculate scale factor" << std::endl;
        calculate_scale_factor(dt, tolerance, goal, cost, output, trajectory, dp);

        // std::cout << "last state: \n" << trajectory.trajectory.back() << std::endl;
        // std::cout << "cost vector: \n" << cost << std::endl;
        // std::cout << "cost: \n" << cost.norm() << std::endl;
        if(cost.norm() > 100){
            std::cout << "cost is too large" << std::endl;
            return -1;
        }
        // std::cout << "dp: \n" << dp << std::endl;
        if((cost.norm() > last_cost) || std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) || std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2)) || fabs(dp(2)) > distance_to_goal){
            if(verbose){
                std::cout << "diverge to infinity!!!" << std::endl;
            }
            return -1;
        }
        last_cost = cost.norm();

        output.omega.km += dp(0);
        output.omega.kf += dp(1);
        output.omega.sf += dp(2);
        // output.omega.km = std::min(std::max(output.omega.km, -MAX_YAWRATE), MAX_YAWRATE);
        // output.omega.kf = std::min(std::max(output.omega.kf, -MAX_YAWRATE), MAX_YAWRATE);
        // std::cout << "output: " << output.omega.km << ", " << output.omega.kf << ", " << output.omega.sf << std::endl;

        // if(fabsf(output.omega.sf - distance_to_goal) > distance_to_goal * 0.5){
        //     if(verbose){
        //         std::cout << "optimization error!!!" << std::endl;
        //     }
        //     return -1;
        // }
        //std::cout << output.omega.km << ", " << output.omega.kf << ", " << output.omega.sf << std::endl;

        // std::cout << "count: " << count << std::endl;
        count++;
    }
    //std::cout << "final cost: \n" << cost << std::endl;
    return cost.norm();
}

double TrajectoryGeneratorDiffDrive::generate_non_optimized_trajectory(const Eigen::Vector3d& goal, const MotionModelDiffDrive::ControlParams& init_control_param, const double dt, const double tolerance, const int max_iteration, MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory)
{
    Eigen::Vector3d cost(1e2, 1e2, 1e2);
    double last_cost = cost.norm();

    double distance_to_goal = goal.segment(0, 2).norm();
    // std::cout << "d_goal: " << distance_to_goal << "[m]" << std::endl;

    output = init_control_param;

        trajectory.trajectory.clear();
        trajectory.velocities.clear();
        trajectory.angular_velocities.clear();
        model.generate_trajectory(dt, output, trajectory);
        cost = goal - trajectory.trajectory.back();
        if(cost.norm() > 2){
            //std::cout << "cost is too large" <<distance_to_goal<< std::endl;
            return -1;
        }
    //std::cout << "final cost: \n" << cost << std::endl;
        return cost.norm();
}

double TrajectoryGeneratorDiffDrive::generate_trajectory_from_param( const MotionModelDiffDrive::ControlParams& init_control_param, MotionModelDiffDrive::Trajectory& trajectory)
{
  trajectory.trajectory.clear();
  trajectory.velocities.clear();
  trajectory.angular_velocities.clear();
  double dt = 0.1;
  model.generate_trajectory(dt, init_control_param, trajectory);
  return 1;
}

double TrajectoryGeneratorDiffDrive::generate_hm_optimized_trajectory(const Eigen::Vector3d& goal, const MotionModelDiffDrive::ControlParams& init_control_param, const double dt, const double tolerance, const int max_iteration, MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory)
{
  setGoal(goal);
  initLamda();
  Eigen::Vector3d cost(1e2, 1e2, 1e2);
  double last_hm_error = cost.norm();
  double distance_to_goal = goal.segment(0, 2).norm();
  output = init_control_param;
  int count = 0;
  double hm_error;
  Eigen::Matrix3d jacobian_;
  Eigen::Matrix3d hessian_;
  Eigen::Vector3d hm_derive_;
  Eigen::Vector3d diff_;

  while(1){
      // std::cout << "---" << std::endl;
      if(cost.norm() < tolerance){
          if(verbose){
              std::cout << "successfully optimized in " << count << " iteration" << std::endl;
          }
          break;
      }else if(count >= max_iteration){
          if(verbose){
              std::cout << "cannot optimize trajectory" << std::endl;
          }
          return -1;
      }
      trajectory.trajectory.clear();
      trajectory.velocities.clear();
      trajectory.angular_velocities.clear();
      model.generate_trajectory(dt, output, trajectory);
      get_opt_para(dt, output, hm_error, h, jacobian_, hessian_, hm_derive_, diff_);
      Eigen::Matrix<double, 6, 6> m6;

      m6 << hessian_(0,0), hessian_(0,1), hessian_(0,2), jacobian_(0,0), jacobian_(1,0), jacobian_(2,0),
            hessian_(1,0), hessian_(1,1), hessian_(1,2), jacobian_(0,1), jacobian_(1,1), jacobian_(2,1),
            hessian_(2,0), hessian_(2,1), hessian_(2,2), jacobian_(0,2), jacobian_(1,2), jacobian_(2,2),
            jacobian_(0,0), jacobian_(0,1), jacobian_(0,2), 0, 0, 0,
            jacobian_(1,0), jacobian_(1,1), jacobian_(1,2), 0, 0, 0,
            jacobian_(2,0), jacobian_(2,1), jacobian_(2,2), 0, 0, 0;

      Eigen::VectorXd v6 = Eigen::VectorXd::Zero(6);
      v6 << hm_derive_[0], hm_derive_[1], hm_derive_[2], diff_[0], diff_[1], diff_[2];
      Eigen::VectorXd dp_ = m6.lu().solve(v6);
      Eigen::Vector3d dp = dp_.segment(0, 3);
      Eigen::Vector3d d_lamda = dp_.segment(3,3);
      //calculate_scale_factor(dt, tolerance, goal, cost, output, trajectory, dp);
      calculate_hm_scale_factor(dt, tolerance, hm_error, goal, diff_, output, trajectory, dp, d_lamda);
      if(hm_error > 100){
          std::cout << "cost is too large" << std::endl;
          return -1;
      }
      // std::cout << "dp: \n" << dp << std::endl;
      if((hm_error > last_hm_error) || std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) || std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2)) || fabs(dp(2)) > distance_to_goal){
          if(verbose){
              std::cout << "diverge to infinity!!!" << std::endl;
          }
          return -1;
      }
      else if ( std::isnan(d_lamda(0)) || std::isnan(d_lamda(1)) || std::isnan(d_lamda(2)) || std::isinf(d_lamda(0)) || std::isinf(d_lamda(1)) || std::isinf(d_lamda(2))){
        return -1;
      }
      last_hm_error = hm_error;
      count++;
  }
  //std::cout << "final cost: \n" << cost << std::endl;
  return last_hm_error;
}

void TrajectoryGeneratorDiffDrive::get_jacobian(const double dt, const MotionModelDiffDrive::ControlParams& control, const Eigen::Vector3d& h, Eigen::Matrix3d& j)
{
    /*
     * h: (dkm, dkf, dsf)
     */
    //std::cout << "j start" << std::endl;
    MotionModelDiffDrive::AngularVelocityParams omega = control.omega;
//    Eigen::Vector3d x_dkm_1;
//    double hm_dkm_1;
//    model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + h(0),omega.kf, hm_dkm_1, x_dkm_1);

    Eigen::Vector3d x0;
    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km - h(0), omega.kf, x0);
    Eigen::Vector3d x1;
    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km + h(0), omega.kf, x1);

    Eigen::Vector3d dx_dkm;
    dx_dkm << (x1 - x0) / (2.0 * h(0));

    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf - h(1), x0);
    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf + h(1), x1);

    Eigen::Vector3d dx_dkf;
    dx_dkf << (x1 - x0) / (2.0 * h(1));

    model.generate_last_state(dt, omega.sf - h(2), control.vel, omega.k0, omega.km, omega.kf, x0);
    model.generate_last_state(dt, omega.sf + h(2), control.vel, omega.k0, omega.km, omega.kf, x1);

    Eigen::Vector3d dx_dsf;
    dx_dsf << (x1 - x0) / (2.0 * h(2));

    j << dx_dkm(0), dx_dkf(0), dx_dsf(0),
         dx_dkm(1), dx_dkf(1), dx_dsf(1),
         dx_dkm(2), dx_dkf(2), dx_dsf(2);
}

void TrajectoryGeneratorDiffDrive::calculate_scale_factor(double dt, double tolerance, const Eigen::Vector3d& goal, Eigen::Vector3d& cost, MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory, Eigen::Vector3d& dp)
{
    for(double beta=0.25;beta<=1.5;beta+=0.25){
        // std::cout << "beta: " << beta << std::endl;
        if(beta == 1.0f){
            continue;
        }
        Eigen::Vector3d dp_ = beta * dp;
        MotionModelDiffDrive::Trajectory trajectory_;
        MotionModelDiffDrive::ControlParams output_ = output;
        output_.omega.km += dp_(0);
        output_.omega.kf += dp_(1);
        output_.omega.sf += dp_(2);
        // std::cout << "dp_: " << dp_.transpose() << std::endl;
        // output_.omega.km = std::min(std::max(output_.omega.km, -MAX_YAWRATE), MAX_YAWRATE);
        // output_.omega.kf = std::min(std::max(output_.omega.kf, -MAX_YAWRATE), MAX_YAWRATE);
        // std::cout << "output: " << output_.omega.km << ", " << output_.omega.kf << ", " << output_.omega.sf << std::endl;
        model.generate_trajectory(dt, output_, trajectory_);
        // std::cout << "size: " << trajectory_.trajectory.size() << std::endl;
        if(trajectory_.trajectory.size() <= 1){
            // std::cout << "failed to generate trajecotry!!!" << std::endl;
            continue;
        }
        Eigen::Vector3d cost_ = goal - trajectory_.trajectory.back();
        // std::cout << cost_.norm() << " vs " << cost.norm() << std::endl;
        if(cost_.norm() < cost.norm()){
            cost = cost_;
            output = output_;
            trajectory = trajectory_;
            dp = dp_;
            // std::cout << "updated!!!" << std::endl;
        }else{
            // std::cout << "not updated!!!" << std::endl;
        }
        if(cost.norm() < tolerance){
            return;
        }
    }
}

void TrajectoryGeneratorDiffDrive::calculate_hm_scale_factor(double dt, double tolerance, double hm_error,const Eigen::Vector3d& goal, Eigen::Vector3d& cost, MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory, Eigen::Vector3d& dp, Eigen::Vector3d& d_lamda)
{
  for(double beta=0.25;beta<=1.5;beta+=0.25){
      // std::cout << "beta: " << beta << std::endl;
      if(beta == 1.0f){
          continue;
      }
      Eigen::Vector3d dp_ = beta * dp;
      Eigen::Vector3d d_lamda_ = beta * d_lamda;
      MotionModelDiffDrive::Trajectory trajectory_;
      MotionModelDiffDrive::ControlParams output_ = output;
      output_.omega.km += dp_(0);
      output_.omega.kf += dp_(1);
      output_.omega.sf += dp_(2);
      double km_ = output_.omega.km;
      double kf_ = output_.omega.kf;
      double sf_ = output_.omega.sf;
      double k0_ = output_.omega.k0;
      Eigen::Vector3d error3_;
      double hamilton_;
      //double v0_ = output_.vel;

      // std::cout << "dp_: " << dp_.transpose() << std::endl;
      // output_.omega.km = std::min(std::max(output_.omega.km, -MAX_YAWRATE), MAX_YAWRATE);
      // output_.omega.kf = std::min(std::max(output_.omega.kf, -MAX_YAWRATE), MAX_YAWRATE);
      // std::cout << "output: " << output_.omega.km << ", " << output_.omega.kf << ", " << output_.omega.sf << std::endl;

      model.get_hamiltonian_from_param(dt, sf_, output_.vel, k0_, km_ , kf_, hamilton_, error3_);
      hamilton_ = hamilton_ + d_lamda_[0] * error3_[0] + d_lamda_[1] * error3_[1] + d_lamda_[2] * error3_[2];
      // std::cout << "size: " << trajectory_.trajectory.size() << std::endl;
      if(trajectory_.trajectory.size() <= 1){
          // std::cout << "failed to generate trajecotry!!!" << std::endl;
          continue;
      }
      //Eigen::Vector3d cost_ = goal - trajectory_.trajectory.back();
      // std::cout << cost_.norm() << " vs " << cost.norm() << std::endl;
      if(hamilton_ < hm_error){
          output = output_;
          trajectory = trajectory_;
          dp = dp_;
          d_lamda = d_lamda_;
          // std::cout << "updated!!!" << std::endl;
      }else{
          // std::cout << "not updated!!!" << std::endl;
      }
      if(hamilton_ < tolerance){
          return;
      }
  }
}


void TrajectoryGeneratorDiffDrive::get_hessian_2derivative(const double dt, const MotionModelDiffDrive::ControlParams& control, const Eigen::Vector3d& h, Eigen::Matrix3d& j)
{
//  Eigen::Vector3d err3_;
//  MotionModelDiffDrive::AngularVelocityParams omega = control.omega;
//  Eigen::Vector3d x_dkm_1;
//  double hm_dkm_1;
//  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + h(0),omega.kf, hm_dkm_1, x_dkm_1);
//  err3_ << x_dkm_1 - _goal;
//  hm_dkm_1 = hm_dkm_1 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dkm_2;
//  double hm_dkm_2;
//  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + 2 * h(0),omega.kf, hm_dkm_2, x_dkm_2);
//  err3_ << x_dkm_2 - _goal;
//  hm_dkm_2 = hm_dkm_2 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];


//  Eigen::Vector3d x_dkf_1;
//  double hm_dkf_1;
//  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf + h(1), hm_dkf_1, x_dkf_1);
//  err3_ << x_dkf_1 - _goal;
//  hm_dkf_1 = hm_dkf_1 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dkf_2;
//  double hm_dkf_2;
//  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf +2 * h(1), hm_dkf_2, x_dkf_2);
//  err3_ << x_dkf_2 - _goal;
//  hm_dkf_2 = hm_dkf_2 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dsf_1;
//  double hm_dsf_1;
//  model.get_hamiltonian_from_param(dt, omega.sf + h(2), control.vel, omega.k0, omega.km, omega.kf, hm_dsf_1, x_dsf_1);
//  err3_ << x_dsf_1 - _goal;
//  hm_dsf_1 = hm_dsf_1 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dsf_2;
//  double hm_dsf_2;
//  model.get_hamiltonian_from_param(dt, omega.sf + 2 * h(2), control.vel, omega.k0, omega.km, omega.kf, hm_dsf_2, x_dsf_2);
//  err3_ << x_dsf_2 - _goal;
//  hm_dsf_2 = hm_dsf_2 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dkm_kf;
//  double hm_dkm_kf;
//  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + h(0), omega.kf + h(1), hm_dkm_kf, x_dkm_kf);
//  err3_ << x_dkm_kf - _goal;
//  hm_dkm_kf = hm_dkm_kf + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dkm_sf;
//  double hm_dkm_sf;
//  model.get_hamiltonian_from_param(dt, omega.sf + h(2), control.vel, omega.k0, omega.km + h(0), omega.kf, hm_dkm_sf, x_dkm_sf);
//  err3_ << x_dkm_sf - _goal;
//  hm_dkm_sf = hm_dkm_sf + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_dkf_sf;
//  double hm_dkf_sf;
//  model.get_hamiltonian_from_param(dt, omega.sf + h(2), control.vel, omega.k0, omega.km, omega.kf + h(1), hm_dkf_sf, x_dkf_sf);
//  err3_ << x_dkf_sf - _goal;
//  hm_dkf_sf = hm_dkf_sf + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Vector3d x_ori;
//  double hm_ori;
//  model.get_hamiltonian_from_param(dt, omega.sf + 2 * h(2), control.vel, omega.k0, omega.km, omega.kf, hm_ori, x_ori);
//  err3_ << x_ori - _goal;
//  hm_ori = hm_ori + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

//  Eigen::Matrix3d hes;
//  hes(0,0) = (hm_dkm_2 - 2 * hm_dkm_1 + hm_ori) / (h(0) * h(0));
//  hes(1,1) = (hm_dkf_2 - 2 * hm_dkf_1 + hm_ori) / (h(1) * h(1));
//  hes(2,2) = (hm_dsf_2 - 2 * hm_dsf_1 + hm_ori) / (h(2) * h(2));
//  hes(1,0) = (hm_dkm_kf - hm_dkm_1 - hm_dkf_1 + hm_ori) /(h(0) * h(1));
//  hes(2,0) = (hm_dkm_sf - hm_dkm_1 - hm_dsf_1 + hm_ori) /(h(0) * h(2));
//  hes(2,1) = (hm_dkf_sf - hm_dkf_1 - hm_dsf_1 + hm_ori) /(h(1) * h(2));
//  hes(0,1) = hes(1,0);
//  hes(0,2) = hes(2,0);
//  hes(1,2) = hes(2,1);


//  Eigen::Vector3d dx_dkm;
//  dx_dkm << (x_dkm_1 - x_ori) / h(0);
//  Eigen::Vector3d dx_dkf;
//  dx_dkf << (x_dkf_1 - x_ori) / h(1);
//  Eigen::Vector3d dx_dsf;
//  dx_dsf << (x_dsf_1 - x_ori) / h(2);

//  Eigen::Matrix3d jac_;
//  jac_ << dx_dkm(0), dx_dkf(0), dx_dsf(0),
//       dx_dkm(1), dx_dkf(1), dx_dsf(1),
//       dx_dkm(2), dx_dkf(2), dx_dsf(2);

//  Eigen::Vector3d hes_deriv;
//  hes_deriv[0] = (hm_dkm_1 - hm_ori) / h(0);
//  hes_deriv[1] = (hm_dkf_1 - hm_ori) / h(1);
//  hes_deriv[2] = (hm_dsf_1 - hm_ori) / h(2);
}

void TrajectoryGeneratorDiffDrive::get_opt_para(const double dt, const MotionModelDiffDrive::ControlParams &control, double& hm_err_,const Eigen::Vector3d &h, Eigen::Matrix3d &jacobi_, Eigen::Matrix3d &hessian_, Eigen::Vector3d &hm_derive_, Eigen::Vector3d &diff_)
{
  Eigen::Vector3d err3_;
  MotionModelDiffDrive::AngularVelocityParams omega = control.omega;
  Eigen::Vector3d x_dkm_1;
  double hm_dkm_1;
  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + h(0),omega.kf, hm_dkm_1, x_dkm_1);
  err3_ << x_dkm_1 - _goal;
  hm_dkm_1 = hm_dkm_1 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dkm_2;
  double hm_dkm_2;
  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + 2 * h(0),omega.kf, hm_dkm_2, x_dkm_2);
  err3_ << x_dkm_2 - _goal;
  hm_dkm_2 = hm_dkm_2 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];


  Eigen::Vector3d x_dkf_1;
  double hm_dkf_1;
  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf + h(1), hm_dkf_1, x_dkf_1);
  err3_ << x_dkf_1 - _goal;
  hm_dkf_1 = hm_dkf_1 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dkf_2;
  double hm_dkf_2;
  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf +2 * h(1), hm_dkf_2, x_dkf_2);
  err3_ << x_dkf_2 - _goal;
  hm_dkf_2 = hm_dkf_2 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dsf_1;
  double hm_dsf_1;
  model.get_hamiltonian_from_param(dt, omega.sf + h(2), control.vel, omega.k0, omega.km, omega.kf, hm_dsf_1, x_dsf_1);
  err3_ << x_dsf_1 - _goal;
  hm_dsf_1 = hm_dsf_1 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dsf_2;
  double hm_dsf_2;
  model.get_hamiltonian_from_param(dt, omega.sf + 2 * h(2), control.vel, omega.k0, omega.km, omega.kf, hm_dsf_2, x_dsf_2);
  err3_ << x_dsf_2 - _goal;
  hm_dsf_2 = hm_dsf_2 + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dkm_kf;
  double hm_dkm_kf;
  model.get_hamiltonian_from_param(dt, omega.sf, control.vel, omega.k0, omega.km + h(0), omega.kf + h(1), hm_dkm_kf, x_dkm_kf);
  err3_ << x_dkm_kf - _goal;
  hm_dkm_kf = hm_dkm_kf + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dkm_sf;
  double hm_dkm_sf;
  model.get_hamiltonian_from_param(dt, omega.sf + h(2), control.vel, omega.k0, omega.km + h(0), omega.kf, hm_dkm_sf, x_dkm_sf);
  err3_ << x_dkm_sf - _goal;
  hm_dkm_sf = hm_dkm_sf + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_dkf_sf;
  double hm_dkf_sf;
  model.get_hamiltonian_from_param(dt, omega.sf + h(2), control.vel, omega.k0, omega.km, omega.kf + h(1), hm_dkf_sf, x_dkf_sf);
  err3_ << x_dkf_sf - _goal;
  hm_dkf_sf = hm_dkf_sf + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Vector3d x_ori;
  double hm_ori;
  model.get_hamiltonian_from_param(dt, omega.sf + 2 * h(2), control.vel, omega.k0, omega.km, omega.kf, hm_ori, x_ori);
  err3_ << x_ori - _goal;
  hm_ori = hm_ori + _lamda[0] * err3_[0] + _lamda[1] * err3_[1] + _lamda[2] * err3_[2];

  Eigen::Matrix3d hes;
  hes(0,0) = (hm_dkm_2 - 2 * hm_dkm_1 + hm_ori) / (h(0) * h(0));
  hes(1,1) = (hm_dkf_2 - 2 * hm_dkf_1 + hm_ori) / (h(1) * h(1));
  hes(2,2) = (hm_dsf_2 - 2 * hm_dsf_1 + hm_ori) / (h(2) * h(2));
  hes(1,0) = (hm_dkm_kf - hm_dkm_1 - hm_dkf_1 + hm_ori) /(h(0) * h(1));
  hes(2,0) = (hm_dkm_sf - hm_dkm_1 - hm_dsf_1 + hm_ori) /(h(0) * h(2));
  hes(2,1) = (hm_dkf_sf - hm_dkf_1 - hm_dsf_1 + hm_ori) /(h(1) * h(2));
  hes(0,1) = hes(1,0);
  hes(0,2) = hes(2,0);
  hes(1,2) = hes(2,1);

  Eigen::Vector3d dx_dkm;
  dx_dkm << (x_dkm_1 - x_ori) / h(0);
  Eigen::Vector3d dx_dkf;
  dx_dkf << (x_dkf_1 - x_ori) / h(1);
  Eigen::Vector3d dx_dsf;
  dx_dsf << (x_dsf_1 - x_ori) / h(2);
  Eigen::Matrix3d jac_;
  jac_ << dx_dkm(0), dx_dkf(0), dx_dsf(0),
       dx_dkm(1), dx_dkf(1), dx_dsf(1),
       dx_dkm(2), dx_dkf(2), dx_dsf(2);

  Eigen::Vector3d hes_deriv;
  hes_deriv[0] = (hm_dkm_1 - hm_ori) / h(0);
  hes_deriv[1] = (hm_dkf_1 - hm_ori) / h(1);
  hes_deriv[2] = (hm_dsf_1 - hm_ori) / h(2);
  hessian_ = hes;
  jacobi_ = jac_;
  hm_derive_ = hes_deriv;
  diff_ = err3_;
  hm_err_ = hm_ori;

}

void TrajectoryGeneratorDiffDrive::setGoal(Eigen::Vector3d goal_)
{
  _goal = goal_;
}

void TrajectoryGeneratorDiffDrive::initLamda()
{
  _lamda << 1, 1, 1;
}

bool TrajectoryGeneratorDiffDrive::setDecisionMaker(decisionMaker* decision_maker_)
{
  model.decision_maker = decision_maker_;
}
