#include <iostream>
#include "rizon_ik_solver.hpp"

#include <random> 

// int main() { 

//     auto kinematics = std::make_shared<FlexivKinematics>(); 

//     auto rizon_nlopt_ik = std::make_shared<RizonNloptIK>();

//     auto current_theta = std::array<double, 7>{0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0};

//     kinematics->update(current_theta); 

//     auto desired_pose = std::vector<double>{0.3, 0.2, 0.6, 0, 1, 0, 0}; 

//     std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now(); 

//     for (int i = 0; i < 30; ++i) { 

//         auto u = rizon_nlopt_ik->get_ik(desired_pose);
//         std::vector<float> delta_theta_float(u.begin(), u.end()); 
//         auto delta_theta_tensor = torch::from_blob(delta_theta_float.data(), {7}, torch::kFloat).clone();

//         auto qq = kinematics->current_q.clone(); 
//         qq += delta_theta_tensor; 
//         kinematics->update(qq); 

//         std::cout << "delta theta: " << std::endl; 
//         for (auto i : u) { std::cout << i << " "; }
//         std::cout << std::endl;
//         std::cout << std::endl;
//     }

//     std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now(); 

//      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0; 
//     std::cout << "Use " << duration << " milliseconds for " << 20 << std::endl;

//     std::cout << kinematics->T_link[-1] << std::endl; 
// }

std::vector<double> generateUnitQuaternion() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double u1 = dis(gen);
    double u2 = dis(gen) * 2.0 * M_PI;
    double u3 = dis(gen) * 2.0 * M_PI;

    double sqrt1MinusU1 = std::sqrt(1 - u1);
    double sqrtU1 = std::sqrt(u1);

    return {
        sqrt1MinusU1 * std::sin(u2), // x
        sqrt1MinusU1 * std::cos(u2), // y
        sqrtU1 * std::sin(u3),       // z
        sqrtU1 * std::cos(u3)        // w (real part)
    };
}

std::vector<double> randomPose() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);
    std::vector<double> vec(7);
    for (int i = 0; i < 3; ++i) {
        vec[i] = dis(gen);
    }
    std::vector<double> quaternion = generateUnitQuaternion();
    for (int i = 0; i < 4; ++i) {
        vec[3 + i] = quaternion[i];
    }
    return vec; 
}



int main() {
    auto ik = RizonIKSolver();
    auto current_theta = std::array<double, 7>{0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0};
    auto desired_pose = std::vector<double>{0.3, 0.2, 0.6, 0, 1, 0, 0}; 
    ik.kinematics->update(current_theta); 

    std::vector<double> increment(7, 0.0); 

    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;

    int times  = 100; 

    std::vector<std::vector<double>> poses;

    for (int i = 0; i < times; ++i) {
        poses.push_back(randomPose()); 
    }

    std::vector<double> nlp_time; 
    std::vector<double> shared_time; 

    for (int t = 0; t < 100; ++t) { 
        startTime = std::chrono::system_clock::now();
        for (int i = 0; i < times; ++i) {
            increment = ik.nlopt_ik(poses[i]);
        }
        endTime = std::chrono::system_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
        std::cout << duration / times  << " milliseconds" <<std::endl; 
        nlp_time.push_back(duration / times); 
    } 

    // for (auto & i : increment) {
    //     std::cout << i << std::endl; 
    // }
    // std::cout << std::endl; 

    for (int t = 0; t < 100; ++t) { 
        startTime = std::chrono::system_clock::now();
        for (int i = 0; i < times; ++i) {
            increment = ik.control_ik(poses[i]); 
        }
        endTime = std::chrono::system_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
        std::cout << duration / times  << " milliseconds" <<std::endl; 
        shared_time.push_back(duration / times); 
    }
    
    std::cout << "==========================" << std::endl; 
    std::vector<double> improvement; 
    for (int t = 0; t < 100; ++t) {
        improvement.push_back((nlp_time[t] - shared_time[t]) / nlp_time[t]); 
        std::cout << (nlp_time[t] - shared_time[t]) / nlp_time[t] << std::endl; 
    }

    // 计算平均值
    double sum = std::accumulate(improvement.begin(), improvement.end(), 0.0);
    double mean = sum / improvement.size();

    double sq_sum = std::accumulate(improvement.begin(), improvement.end(), 0.0,
        [mean](double a, double b) {
            return a + std::pow(b - mean, 2);
        });

    double variance = sq_sum / improvement.size();
    double std_dev = std::sqrt(variance);

    std::cout << "平均值 = " << mean << std::endl;
    std::cout << "标准差 = " << std_dev << std::endl;

}