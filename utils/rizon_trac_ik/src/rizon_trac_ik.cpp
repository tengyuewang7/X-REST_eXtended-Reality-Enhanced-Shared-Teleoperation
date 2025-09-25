#include "rizon_trac_ik.hpp"


using namespace std::chrono_literals;


void PrintChainDetails(const KDL::Chain& chain); 


RizonTracIK::RizonTracIK() {
	std::string chain_start = "base_link";
	std::string chain_end = "flange";
	std::string urdf_xml;
	double timeout = 1.0;

	std::string data_path = "/home/rvc/colcon_ws/utils/rizon_trac_ik/src/urdf_xml.txt"; 

	// std::string data_path = "/home/rvc/colcon_ws/src/to_unity/to_unity_cpp/launch/rizon4_modified.urdf"; 
	std::ifstream inFile(data_path);
	if (inFile.is_open()) {
		std::stringstream buffer;
		buffer << inFile.rdbuf();
		urdf_xml = buffer.str();
		inFile.close();
	}
	else { std::cout << "Fail to open urdf_xml.txt"; }

	double eps = 1e-5; // error
	tracik_solver_ptr = std::make_shared<TRAC_IK::TRAC_IK>(chain_start, chain_end, urdf_xml, timeout, eps);

	KDL::JntArray ll, ul; // lower joint limits, upper joint limits
	KDL::Chain chain;

	// Check if KDL chain, lower joint limits and upper joint limits are valid. 
	if (!(tracik_solver_ptr->getKDLChain(chain)) || !(tracik_solver_ptr->getKDLLimits(ll, ul))) {
		std::cout << "There are no valid KDL chain or ll/ul found";
	}

	// PrintChainDetails(chain); 

	fk_solver_ptr = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);

	// Set default initial joint state at the home position [0, -40, 0, 90, 0, 40, 0] in degree. 
	nominal = std::make_shared<KDL::JntArray>(chain.getNrOfJoints());
	(*nominal)(0) = 0.0;
	(*nominal)(1) = -0.698132;  // -40 degree
	(*nominal)(2) = 0.0;
	(*nominal)(3) = 1.570796;   // 90 degree
	(*nominal)(4) = 0.0;
	(*nominal)(5) = 0.698132;   // 40 degree
	(*nominal)(6) = 0.0; 
	result = std::make_shared<KDL::JntArray>(chain.getNrOfJoints()); 
	(*result) = (*nominal);
}

std::pair<double, std::vector<double>> RizonTracIK::get_ik(std::vector<double> current_q, std::vector<double> desired_pose, char return_type) {
	for (int i = 0; i < 7; i++) {
		(*nominal)(i) = current_q[i] + 1e-3;
	}

	KDL::Vector v(desired_pose[0], desired_pose[1], desired_pose[2]);
	KDL::Rotation r = KDL::Rotation::Quaternion(
		desired_pose[3], desired_pose[4], 
		desired_pose[5], desired_pose[6]);
	
	const KDL::Frame pose(r, v);


	int rc = tracik_solver_ptr->CartToJnt(*nominal, pose, *result); 

	std::vector<double> q(current_q.begin(), current_q.end());

	if (rc >= 0) { 
		for (int i = 0; i < 7; ++i) {
			q[i] = (*result)(i);
		}
	}

	if (return_type == 'u') {
		for (int i = 0; i < 7; ++i) {
			q[i] -= current_q[i]; 
		}
	}

	return std::make_pair(rc, q);
}

std::vector<double> RizonTracIK::get_flange_pose(std::vector<double> current_q) { 
	KDL::JntArray q(current_q.size());
	for (size_t i = 0; i < current_q.size(); ++i) {
        q(i) = current_q[i];
    }
	KDL::Frame flange_pose; 

	std::cout << q.data << std::endl; 
	std::cout << fk_solver_ptr->JntToCart(q, flange_pose) << std::endl; 

	double rx, ry, rz, rw; 
	flange_pose.M.GetQuaternion(rx, ry, rz, rw); 

	return std::vector<double>{flange_pose.p.x(), flange_pose.p.y(), flange_pose.p.z(), 
							   rx, ry, rz, rw }; 
}

void PrintChainDetails(const KDL::Chain& chain) {
    std::cout << "The chain has " << chain.getNrOfSegments() << " segments." << std::endl;

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain.getSegment(i);
        const KDL::Joint& joint = segment.getJoint();

        std::cout << "Segment " << i << ":" << std::endl;
        std::cout << "  Joint Type: " << joint.getTypeName() << std::endl;
        std::cout << "  Joint Name: " << joint.getName() << std::endl;
        std::cout << "  Frame (Transformation from the previous joint): " << std::endl;
    }
}

