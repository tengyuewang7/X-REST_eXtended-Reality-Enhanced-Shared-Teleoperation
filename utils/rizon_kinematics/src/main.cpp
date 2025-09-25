#include "tensor_utils.hpp"
#include "rizon_kinematics.hpp"
#include <random>
#include <iostream>

int main() {

	auto kinematics = RizonKinematics(); 

	auto home = std::vector<double>{0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0}; 

	auto joints = std::array<double, 7>{0.7981, -0.176127, 0.999173, 
                                        0.8532217, -0.1653, 0.4746461, 0.5383144}; 
	
	auto target_pose = std::array<double, 7>{0.4597, -0.4962, 0.581734, 
											0.824, -0.353, 0.243, 0.3706}; 

	kinematics.update(joints); 

	kinematics.update(std::vector<double>{1.4888, -0.337447, -0.0305448, 1.22867, 0.434522, 1.03156, 0.0696426}); 

	std::cout << kinematics.get_flange_pose() << std::endl;

	kinematics.solve_ik(target_pose); 

	std::cout << kinematics.get_current_q_vector() << std::endl;
	std::cout << kinematics.get_flange_pose() << std::endl;

	// std::cout << kinematics.T_joint << std::endl;  
	// std::cout << kinematics.jaco_eff << std::endl;  
	
	kinematics.update_dpose(target_pose); 
	// std::cout << kinematics.dpose << std::endl; 

	// std::cout << kinematics.control_ik(target_pose) << std::endl; 

    std::vector<std::vector<double>> obs = std::vector<std::vector<double>>{
        {0.4726,  -0.8524,  0.7505},
        {0.5982,  0.6531,  -0.7944},
        {0.1531,  0.8410,  0.5192},
        {-0.3586,  0.9772,  -0.2563},
        {-0.3816,  0.5886,  0.3035},
        {0.9168,  -0.9232,  -0.8427},
        {0.7059,  0.2137,  0.1104},
        {0.4597,  -0.1861,  0.9977},
        {-0.3821,  -0.8047,  0.0657},
        {0.0,  0.3135, 1.05}
	};
    kinematics.update_obstacles(obs); 

	auto start = std::chrono::high_resolution_clock::now(); 

	for (int i = 0; i < 1000; ++i) {
		torch::Tensor random_tensor = torch::rand({7}) * 2 * M_PI - M_PI; 
		kinematics.update(random_tensor); 
		kinematics.control_ik(target_pose); 
	}
	
	auto stop = std::chrono::high_resolution_clock::now();
	
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Function took " << duration.count() / 1e3 << " milliseconds." << std::endl;


	return 0; 

} 

