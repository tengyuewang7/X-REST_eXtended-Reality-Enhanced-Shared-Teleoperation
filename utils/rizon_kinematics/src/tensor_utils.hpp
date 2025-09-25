#ifndef TENSOR_UTILS_HPP
#define TENSOR_UTILS_HPP 

#include <torch/torch.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tensor_utils
{

const torch::Device device = torch::kCPU;
const torch::TensorOptions tensor_options = torch::TensorOptions().dtype(at::kFloat).device(device); 


/** 
 * @brief Transform convert a rotation matrix to a quaternion
 * @param matrix {3, 3} tensor, rotation matrix 
 * @return {4} tensor, a quaternion in [rx, ry, rz, rw]
 */
inline Eigen::MatrixXd tensor_matrix_to_eigen_matrix(const torch::Tensor& matrix) {  

    auto tensor = matrix.to(torch::kCPU).toType(torch::kFloat); 

    Eigen::MatrixXd result(tensor.size(0), tensor.size(1)); 
    for (int i = 0; i < tensor.size(0); ++i) {
        for (int j = 0; j < tensor.size(1); ++j) {
            result(i, j) = tensor[i][j].item<float>();
        }
    }

    return result;
}

/** 
 * @brief Transform convert a rotation matrix to a quaternion
 * @param matrix {3, 3} tensor, rotation matrix 
 * @return {4} tensor, a quaternion in [rx, ry, rz, rw]
 */
inline torch::Tensor tensor_matrix_to_tensor_quaternion(const torch::Tensor& matrix) { 
    torch::Tensor quat = torch::empty(4, tensor_options); 
    quat[3] = torch::sqrt(1 + matrix[0][0] + matrix[1][1] + matrix[2][2]) / 2.0; 
    quat[0] = (matrix[2][1] - matrix[1][2]) / (4.0 * quat[3]); 
    quat[1] = (matrix[0][2] - matrix[2][0]) / (4.0 * quat[3]); 
    quat[2] = (matrix[1][0] - matrix[0][1]) / (4.0 * quat[3]); 

    return quat / torch::norm(quat); 
}

/**
 * @brief Convert 2D tensor transformation matrix to a 1D tensor 
 * @param matrix {4, 4} tensor, homogeneous transformation matrix 
 * @return {7} tensor [px, py, pz, rx, ry, rz, rw] 
 */
inline torch::Tensor tensor_matrix_to_tensor_vector(const torch::Tensor& matrix) {
    torch::Tensor rot = tensor_matrix_to_tensor_quaternion(matrix.slice(0, 0, 3).slice(1, 0, 3)); 
    torch::Tensor pos = matrix.slice(0, 0, 3).slice(1, 3, 4).squeeze(-1); 

    return torch::cat({pos, rot}, 0);  
}

/**
 * @brief Convert 1D tensor to std::vector<double> 
 * @param tensor {n} 1D tensor
 * @return size n std::vector<double>
 */
inline std::vector<double> tensor_to_vector(const torch::Tensor& tensor) {
    // Make sure tensor on CPU with double elements 
    auto tensor_cpu_double = tensor.to(torch::kCPU).to(torch::kDouble); 
    // Get pointer of the tensor 
    double* data_ptr = tensor_cpu_double.data_ptr<double>(); 
    // Initialize the return vector 
    std::vector<double> vec(data_ptr, data_ptr + tensor_cpu_double.numel()); 

    return vec; 
}

/**
 * @brief Convert std::vector<double> to 1D tensor 
 * @param  vector n std::vector<double> 
 * @return 1D tensor
 */
template<typename Container>
inline torch::Tensor container_to_tensor(const Container& data) {
    std::vector<float> float_vec(data.begin(), data.end());
    // Warning: from_blob does not take ownership, use clone
    torch::Tensor tensor = torch::from_blob(float_vec.data(), 
                                            {static_cast<int64_t>(float_vec.size())}, 
                                            torch::kFloat).clone().to(device);
    return tensor; 
}



inline torch::Tensor vector2d_to_tensor(const std::vector<std::vector<double>>& vector2d) {
    // Handle empty case: return an empty tensor 
    if (vector2d.empty() || vector2d[0].empty()) {
        return torch::empty({0, 0}, tensor_options);
    }

    // Get the size of the 2D vector 
    signed long rows = vector2d.size(); 
    signed long cols = vector2d[0].size(); 

    // Flat the 2D vector
    std::vector<double> flat_vec;
    for (const auto& row_vector : vector2d) {
        flat_vec.insert(flat_vec.end(), row_vector.begin(), row_vector.end());
    }

    // Create a tensor from the flat std::vector and convert to float tensor
    return torch::from_blob(flat_vec.data(), {rows, cols}, torch::dtype(torch::kDouble)).clone().to(torch::kFloat);
}


/** 
 * @brief Calculate the muliplication of two tensor quaternion  
 * @param quat_1 {4} tensor in [rx, ry, rz, rw] 
 * @param quat_2 {4} tensor in [rx, ry, rz, rw] 
 * @return {4} tensor in [rx, ry, rz, rw] 
 */
inline torch::Tensor quaternion_multiply(const torch::Tensor& quat_1, const torch::Tensor& quat_2, bool norm = true) {
    // Check the size 
    if (quat_1.sizes().size() != 1 || quat_1.sizes().size() != 1 || quat_2.size(0) != 4 || quat_2.size(0) != 4) {
        throw std::runtime_error("Both q1 and q2 must be tensors of size 4");
    }

    // Extract elements  
    auto x1 = quat_1[0];
    auto y1 = quat_1[1];
    auto z1 = quat_1[2];
    auto w1 = quat_1[3];
    auto x2 = quat_2[0];
    auto y2 = quat_2[1];
    auto z2 = quat_2[2];
    auto w2 = quat_2[3];

    // Calculate the result quaternion 
    torch::Tensor res = torch::zeros({4}, tensor_options); 
    res[0] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    res[1] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    res[2] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; 
    res[3] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

    if (norm) { return res / torch::norm(res); }
    else { return res; }
} 

template <typename Container>
inline torch::Tensor quaternion_multiply(const Container& quat1, const Container& quat2, bool norm = true) { 
    return quaternion_multiply(container_to_tensor(quat1), container_to_tensor(quat2), norm); 
}

inline torch::Tensor quaternion_conjugate(const torch::Tensor& quat) {
    return torch::cat({-quat.slice(0, 0, 3), quat.slice(0, 3, 4)}, 0);
}

inline torch::Tensor normalize_angles(torch::Tensor angles) {
    return torch::atan2(torch::sin(angles), torch::cos(angles));
}


}
#endif 
