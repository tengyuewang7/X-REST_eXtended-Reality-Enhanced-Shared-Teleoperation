#ifndef RIZON_TRAC_IK_HPP
#define RIZON_TRAC_IK_HPP

#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <cmath> 
#include <algorithm>
#include <thread> 
#include <trac_ik/trac_ik.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

class RizonTracIK {
public:
    RizonTracIK();

    std::pair<double, std::vector<double>> get_ik(std::vector<double>, 
                                                  std::vector<double>, 
                                                  char return_type='q');
    std::vector<double> get_flange_pose(std::vector<double>); 

private:
    std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver_ptr;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_ptr; 
    std::shared_ptr<KDL::JntArray> nominal;
    std::shared_ptr<KDL::JntArray> result; 
};

#endif 