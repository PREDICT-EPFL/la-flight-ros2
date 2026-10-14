#include <Eigen/Eigen>

#include "model_common/QuaternionMathCasadi.hpp"
#include "model_common/QuaternionMathEigen.hpp"

int main()
{
    using namespace model_common;

    std::vector<double> q1_ab = {1, 0, 0, 0};
    std::vector<double> q2_ab = {0.9, 0, 0.1, 0};
    std::vector<double> q3_ab = casadi::DM(quatmath_casadi::T2quat(10.0 * M_PI / 180.0)).nonzeros();
    Eigen::Quaternion<double> q4_ab_ = quatmath::T2quat<double>(10.0 * M_PI / 180.0);
    std::vector<double> q4_ab = {q4_ab_.w(), q4_ab_.x(), q4_ab_.y(), q4_ab_.z()};

    std::vector<std::vector<double>> quats = {q2_ab, q3_ab};

    std::vector<double> b_v1 = {1, 0, 0};

    unsigned i{0};
    for (auto q: quats)
    {
        i++;
        std::cout << i << "\n";

        // Eigen test
        Eigen::Quaternion<double> qeig_ab(q[0], q[1], q[2], q[3]);

        Eigen::Vector3d b_veig(b_v1.data());
        Eigen::Vector3d a_veig = qeig_ab * b_veig;

        // Casadi test
        casadi::DM qcas_ab(q);

        casadi::DM b_vcas(b_v1);
        casadi::DM a_vcas = quatmath_casadi::quat_transform(qcas_ab, b_vcas);

        std::cout << "Eigen quat qeig_ab: " << qeig_ab.w() << " " << qeig_ab.vec().transpose() << ", norm: " << qeig_ab.norm() << "\n";
        std::cout << "Casad quat qcas_ab: " << qcas_ab << ", norm: " << casadi::DM::norm_2(qcas_ab) << "\n";
        std::cout << "Eigen vec a_vec: " << a_veig.transpose() << ", norm: " << a_veig.norm() << "\n";
        std::cout << "Casadi vec a_vec: " << a_vcas << ", norm: " << casadi::DM::norm_2(a_vcas) << "\n";

        std::cout << "\n";
    }

    return 0;
}
