#include "rizon_ik_solver.hpp" 

RizonIKSolver::RizonIKSolver() : 
    kinematics(std::make_shared<RizonKinematics>()), 
    trac_ik_solver(std::make_shared<RizonTracIK>()), 
    opt(std::make_shared<nlopt::opt>(nlopt::LD_MMA, 7)),
    params(std::make_shared<opt_data>()) {

    opt->set_min_objective(RizonIKSolver::wrap_obj_fun, this); 

    opt->set_lower_bounds(std::vector<double>(7, -0.5)); 
    opt->set_upper_bounds(std::vector<double>(7, 0.5));
    // opt.set_xtol_rel(1e-2);
    // opt->set_maxeval(3); 
    opt->set_maxtime(0.0008);
    // 设置收敛容限
    opt->set_ftol_rel(1e-3); // 更松的相对变化阈值
    opt->set_ftol_abs(1e-6); // 更松的绝对变化阈值 

    delta_theta = std::vector<double>(7, 0.0); 

} 

double RizonIKSolver::ik_objective(const std::vector<double> &delta_theta, std::vector<double> &grad) {
    torch::Tensor j_ee = params->j_ee;
    torch::Tensor dpose = params->dpose; 
    torch::Tensor j_cp = params->j_cp;
    torch::Tensor v_c = params->v_c; 

    std::vector<float> delta_theta_float(delta_theta.begin(), delta_theta.end()); 
    auto delta_theta_tensor = torch::from_blob(delta_theta_float.data(), {7}, torch::kFloat).clone();

    torch::Tensor diff = torch::matmul(j_ee, delta_theta_tensor) - dpose; 
    double objective_1 = torch::sum(diff * diff).item<double>(); 
    double mu = std::pow(0.1, 2); 
    double objective_2 = mu * torch::sum(delta_theta_tensor * delta_theta_tensor).item<double>();
    // v_c has been scaled 
    double objective_3 = 2 * torch::dot(j_cp.matmul(delta_theta_tensor), v_c).item<double>(); 

    double objective_value = objective_1 + objective_2 + objective_3; 

    if (!grad.empty()) {
        auto j_ee_T = torch::transpose(j_ee, 0, 1); 
        torch::Tensor grad_tensor_1 = 2 * torch::matmul(j_ee_T, diff); 
        torch::Tensor grad_tensor_2 = 2 * mu * delta_theta_tensor;  
        torch::Tensor grad_tensor_3 = 2 * torch::transpose(j_cp, 0, 1).matmul(v_c);  
        auto grad_tensor = grad_tensor_1 + grad_tensor_2 + grad_tensor_3; 
        grad = std::vector<double>(7);
        for (int i = 0; i < 7; ++i) { grad[i] = grad_tensor[i].item<double>(); }
    } 
    return objective_value;
} 

std::vector<double> RizonIKSolver::nlopt_ik(std::vector<double> desired_pose) {
    kinematics->update_dpose(desired_pose); 
    std::pair<torch::Tensor,torch::Tensor> close_info; 
    if (kinematics->is_using_meshes) {
        close_info = kinematics->closest_to_links_based_on_meshes(); 
    }
    else {
        close_info = kinematics->closest_to_links(); 
    }
    params->j_ee = kinematics->jaco_eff; 
    params->dpose = kinematics->dpose;
    params->j_cp = close_info.first; 
    params->v_c = close_info.second; 
    nlopt::result result = opt->optimize(delta_theta, minf); 
    std::vector<double> ret(delta_theta.begin(), delta_theta.end()); 

    auto maxIt = std::max_element(ret.begin(), ret.end(), [](double a, double b) { 
        return std::abs(a) < std::abs(b);
    }); 
    double maxAbs = std::abs(*maxIt); 
    if (maxAbs > 0.01) {
        double scale = 0.01 / maxAbs;
        for (auto& val : ret) {
            val *= scale;
        }
    }

    return ret;
}


std::vector<double> RizonIKSolver::control_ik(std::vector<double> desired_pose) {
    return kinematics->control_ik(desired_pose); 
}

std::pair<double, std::vector<double>> RizonIKSolver::trac_ik(std::vector<double> desired_pose) {
    return trac_ik_solver->get_ik(kinematics->get_current_q_vector(), desired_pose, 'u'); 
}

