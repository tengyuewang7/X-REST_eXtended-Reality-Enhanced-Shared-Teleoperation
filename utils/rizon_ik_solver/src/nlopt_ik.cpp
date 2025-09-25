#include <iostream>
#include <cmath>
#include <nlopt.hpp>
#include <vector>
#include <torch/torch.h>

#include "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/libtorch/src/flexiv_kinematics.hpp" 

struct opt_data {
    torch::Tensor j_ee;
    torch::Tensor dpose;
    torch::Tensor j_cp;
    torch::Tensor v_c;
};



double inverse_kinematics_objective(const std::vector<double> &delta_theta, std::vector<double> &grad, void *data) {

    auto* params = static_cast<opt_data*>(data);
    
    torch::Tensor j_ee = params->j_ee;
    torch::Tensor dpose = params->dpose; 
    torch::Tensor j_cp = params->j_cp;
    torch::Tensor v_c = params->v_c; 

    std::vector<float> delta_theta_float(delta_theta.begin(), delta_theta.end()); 
    auto delta_theta_tensor = torch::from_blob(delta_theta_float.data(), {7}, torch::kFloat).clone();

    double mu = std::pow(0.05, 2); 

    torch::Tensor diff = torch::matmul(j_ee, delta_theta_tensor) - dpose; 

    double objective_1 = torch::sum(diff * diff).item<double>(); // + mu * torch::sum(delta_theta_tensor * delta_theta_tensor).item<double>(); 
    double objective_2 = mu * torch::sum(delta_theta_tensor * delta_theta_tensor).item<double>(); 
    double objective_3 = 2 * torch::sum(j_cp.matmul(delta_theta_tensor) * v_c).item<double>();

    double objective_value = objective_1 + objective_2 + objective_3;

    if (!grad.empty()) {
        auto j_ee_trans = torch::transpose(j_ee, 0, 1);

        torch::Tensor grad_tensor_1 = 2 * torch::matmul(j_ee_trans, diff);
        torch::Tensor grad_tensor_2 = 2 * mu * delta_theta_tensor;  
        torch::Tensor grad_tensor_3 = 2 * torch::transpose(j_cp, 0, 1).matmul(v_c);  
        auto grad_tensor = grad_tensor_1 + grad_tensor_2 + grad_tensor_3; 

        grad = std::vector<double>(7);
        for (int i = 0; i < 7; ++i) { grad[i] = grad_tensor[i].item<double>(); }
    }

    return objective_value;
}

int main() {

    auto kinematics = FlexivKinematics(); 

    auto current_theta = std::array<double, 7>{0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0};

    // current_theta = std::array<double, 7>{0.387542, 1.30949, 0.0661437, 1.46461, -0.0517187, 0.152264, 0.47028};
    kinematics.update(current_theta); 
    nlopt::opt opt(nlopt::LD_LBFGS, 7); 
    opt_data params; 
    opt.set_min_objective(inverse_kinematics_objective, &params); 
    std::vector<double> lb(7, -0.1);
    std::vector<double> ub(7, 0.1);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    // opt.set_xtol_rel(1e-2);
    // opt.set_maxeval(1); 
    opt.set_maxtime(0.0005);
    // 设置收敛容限
    opt.set_ftol_rel(1e-3); // 更松的相对变化阈值
    opt.set_ftol_abs(1e-6); // 更松的绝对变化阈值

    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now(); 
    int loop_num = 20;
    for (int loop = 0; loop < loop_num; ++loop) { 
        auto desired_pose = std::vector<double>{0.3, 0.2, 0.6, 0, 1, 0, 0}; 
        kinematics.update_dpose(desired_pose); 

        auto close_info = kinematics.closest_to_links(); 
        params.j_ee = kinematics.jaco_eff;
        params.dpose = kinematics.dpose;
        params.j_cp = close_info.first; 
        params.v_c = close_info.second; 

        static std::vector<double> delta_theta = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        double minf; 

        nlopt::result result = opt.optimize(delta_theta, minf); 

        auto qq = kinematics.current_q.clone();

        std::vector<float> delta_theta_float(delta_theta.begin(), delta_theta.end()); 
        auto delta_theta_tensor = torch::from_blob(delta_theta_float.data(), {7}, torch::kFloat).clone();

        qq += delta_theta_tensor;
        kinematics.update(qq); 

        // std::cout << "delta theta: " << std::endl; 
        // for (auto i : delta_theta) { std::cout << i << " "; }
        // std::cout << std::endl;
        // std::cout << std::endl;

    }

    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now(); 

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0; 
    std::cout << "Use " << duration << " milliseconds for " << loop_num << std::endl;

    // std::cout << kinematics.current_q << std::endl;
    std::cout << kinematics.T_link[-1] << std::endl;

    auto kinematics_2 = FlexivKinematics(); 
    kinematics_2.update(current_theta); 
    auto desired_pose = std::vector<double>{0.3, 0.2, 0.6, 0, 1, 0, 0}; 
    auto res = kinematics_2.direct_ik(desired_pose);
    std::cout << "direct ik: " << std::endl; 
    for (auto i : res) { std::cout << i << " "; }
    std::cout << std::endl;

    auto u_all = kinematics_2.control_ik(desired_pose); 
    std::cout << "control ik: " << std::endl; 
    for (auto i : u_all) { std::cout << i << " "; }
    std::cout << std::endl;
    auto pair = kinematics_2.closest_to_links();





    // kinematics_2.solve_ik(desired_pose);
    // std::cout << kinematics_2.current_q << std::endl;


    return 0;
}
