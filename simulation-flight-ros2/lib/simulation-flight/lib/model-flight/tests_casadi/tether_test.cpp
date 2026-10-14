#include "model_flight/AweDynamicsCasadi.hpp"

int main()
{
    using namespace casadi;

    double teth_density = 1.15;
    double teth_diameter = 10e-3;
    double teth_E = 9e7;
    double teth_len0 = 10;
    flight_model::awe_casadi::Tether teth(teth_len0, teth_diameter, teth_density, teth_E, 1.2);

    DM pos0 = DM::vertcat({0, 0, 0});
    DM vel0 = DM::vertcat({5, 0, 3});
    DM pos1 = DM::vertcat({0, 10.3, 0});
    DM vel1 = DM::vertcat({10, 0, 1});
    DM vW = DM::vertcat({0, 0, 0});

    SX n_T_at_1, n_T_at_0;
    teth.get_F_lon(pos0, pos1,
                   n_T_at_1, n_T_at_0);

    SX n_W_at_1, n_W_at_0;
    teth.get_F_weight(pos0, pos1,
                      n_W_at_1, n_W_at_0);

    SX n_D_at_1, n_D_at_0;
    teth.get_F_drag(pos0, pos1, vel0, vel1, vW,
                    n_D_at_1, n_D_at_0);

    SX n_D_at_1s, n_D_at_0s;
    teth.get_F_drag_simpler(pos0, pos1, vel0, vel1, vW,
                            n_D_at_1s, n_D_at_0s);

    std::cout << std::setprecision(8)
            << "nT1: " << DM(n_T_at_1) << ", nT0: " << DM(n_T_at_0) << "\n"
            << "nW1: " << DM(n_W_at_1) << ", nW0: " << DM(n_W_at_0) << "\n"
            << "nD1: " << DM(n_D_at_1) << ", nD0: " << DM(n_D_at_0) << "\n"
            << "nD1s: " << DM(n_D_at_1s) << ", nD0s: " << DM(n_D_at_0s) << "\n";


    teth.get_forces(pos0, pos1, vel0, vel1, vW,
                    n_T_at_1, n_T_at_0, n_W_at_1, n_W_at_0, n_D_at_1, n_D_at_0);

    std::cout << std::setprecision(8)
            << "nT1: " << DM(n_T_at_1) << ", nT0: " << DM(n_T_at_0) << "\n"
            << "nW1: " << DM(n_W_at_1) << ", nW0: " << DM(n_W_at_0) << "\n"
            << "nD1: " << DM(n_D_at_1) << ", nD0: " << DM(n_D_at_0) << "\n";

    return 0;
}
