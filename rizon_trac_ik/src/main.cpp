#include "rizon_trac_ik.hpp"

using namespace std::chrono_literals;


int main() { 

	auto ik = RizonTracIK();

	auto joints = std::vector<double>{0.7981, -0.176127, 0.999173, 
                                        0.8532217, -0.1653, 0.4746461, 0.5383144}; 

	auto target_pose = std::vector<double>{0.0851109+0.1, 0.493426+0.2, 
							1.03444-0.3, -0.464525, 0.718405, 0.467639, 0.222315}; 

	joints = std::vector<double>{0.0, -40.0/180.0*3.1415926, 0.0, 90.0/180.0*3.1415926, 
									0.0, 40.0/180.0*3.1415926, 0.0}; 

	auto res = ik.get_flange_pose(joints);  

	auto pose = std::vector<double>{ 0.688165, -0.112056, 0.291719, 
									0.0, 0.0, 1.0, 0.0};

	res = ik.get_ik(joints, pose, 'q').second; 

	for (auto &i : res) { std::cout << i << " "; }
	std::cout << std::endl; 

	pose = std::vector<double>{ 0.688165, -0.112056, 0.291719, 
								0.0, 1,0, 0.0, 0.0 };

	res = ik.get_ik(joints, pose, 'u').second; 

	for (auto &i : res) { std::cout << i / 3.1415925 * 180.0 << " "; }
	std::cout << std::endl; 

	return 0;
}
