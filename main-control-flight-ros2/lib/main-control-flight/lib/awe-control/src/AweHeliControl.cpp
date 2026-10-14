#include "awe_control/AweHeliControl.hpp"

using namespace awe_control3;

Eigen::VectorXd AweHeliController::get_normlifts(const std::vector<Eigen::Vector3d> &n_kite_posns,
                                                 const Eigen::Vector2d &d_liftdir)
{
    /* Average over kite positions and compute disk frame */
    Vector3 n_kites_avg_pos(0, 0, 0);
    for (const Eigen::Vector3d &n_kite_pos: n_kite_posns) { n_kites_avg_pos += n_kite_pos / n_kite_posns.size(); }
    const Quaternion q_dn = get_q_dn(n_kites_avg_pos);

    /* User commands (trim and control) in disk frame */
    const double d_nlift_ctrl_arg = std::atan2(d_liftdir(1), d_liftdir(0));
    const double d_nlift_ctrl_mag = d_liftdir.norm();
//    std::cout << "d_liftdir: (ctrl) " << d_liftdir.transpose() << " (arg) " << d_nlift_ctrl_arg * 180.0/M_PI << " (mag) " << d_nlift_ctrl_mag << "\n";

    /* Compute individual kite normalized lift command as a function of bearing in disk frame */
    Eigen::VectorXd normlift_cmds(n_kite_posns.size());
    for (unsigned k = 0; k < n_kite_posns.size(); k++)
    {
        Eigen::Vector3d d_kite_pos = q_dn * n_kite_posns[k];
        const double d_kite_brg = awe_control3::getBearingFromCenter(d_kite_pos.segment<2>(0));

        /* Normalized lift command = trim component + control component */
        const double ctrl_cmd = -d_nlift_ctrl_mag * std::cos(d_kite_brg - d_nlift_ctrl_arg); // Normalized lift control shall be maximal at d_nlift_ctrl_arg
        normlift_cmds(k) = ctrl_cmd;
//        std::cout << "k" << k << ": (coll) " << collective << ", (brg) " << d_kite_brg * 180.0/M_PI << ", (cmd) " << normlift_cmds(k) << "\n";
    }
    return normlift_cmds;
}
