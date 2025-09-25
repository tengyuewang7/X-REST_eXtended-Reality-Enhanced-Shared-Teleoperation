#ifndef RIZON_IK_SOLVER_HPP
#define RIZON_IK_SOLVER_HPP 

#include <iostream>
#include <cmath>
#include <nlopt.hpp>
#include <vector>
#include <torch/torch.h> 
#include "/home/rvc/colcon_ws/utils/rizon_kinematics/src/rizon_kinematics.hpp"
#include "/home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.hpp" 

class RizonIKSolver {

public: 
    struct opt_data
    {
        torch::Tensor j_ee;
        torch::Tensor dpose;
        torch::Tensor j_cp;
        torch::Tensor v_c;
    }; 

    RizonIKSolver(); 
    double ik_objective(const std::vector<double> &, std::vector<double> &);
    std::vector<double> nlopt_ik(std::vector<double>); 
    std::vector<double> control_ik(std::vector<double>); 
    std::pair<double, std::vector<double>> trac_ik(std::vector<double>);



// private:
    std::shared_ptr<RizonKinematics> kinematics;
    std::shared_ptr<RizonTracIK> trac_ik_solver; 
    std::shared_ptr<nlopt::opt> opt; 
    std::shared_ptr<opt_data> params; 
    std::vector<double> delta_theta; 
    double minf;
    static double wrap_obj_fun(const std::vector<double> &x, std::vector<double> &grad, void *data) {
        // 将void* data转换为MyClass*
        RizonIKSolver *instance = static_cast<RizonIKSolver*>(data);
        // 调用成员函数
        return instance->ik_objective(x, grad);
    }

};



#endif 