#include "flight_model/eigen_model/FixedWingDynamicsEigen.hpp"
#include "flight_model/casadi_model/FixedWingDynamicsCasadi.hpp"

#include "polympc/utils/helpers.hpp"
#include "polympc/solvers/nlproblem.hpp"
#include "polympc/solvers/sqp_base.hpp"
#include "polympc/polymath.h"

using DynamicsModel = flight_model::eigen_model::fixed_wing::LonFixedWingDynamics;
//using DynamicsModel = flight_model::eigen_model::fixed_wing::FixedWingDynamics;

POLYMPC_FORWARD_NLP_DECLARATION(TrimProblem, /*NX*/4 + 2, /*NE*/0, /*NI*/0, /*NP*/0, double)
class TrimProblem : public ProblemBase<TrimProblem>
{
public:
    static constexpr int nx = DynamicsModel::nx;
    static constexpr int nu = DynamicsModel::nu;
    DynamicsModel dynamicsModel;

    template<typename T>
    EIGEN_STRONG_INLINE void cost_impl(const Eigen::Ref<const variable_t <T>> &x,
                                       const Eigen::Ref<const static_parameter_t> &p,
                                       T &cost) const noexcept
    {
        polympc::ignore_unused_var(p);

        DynamicsModel::state_t <T> state = x.template segment<nx>(0);
        DynamicsModel::control_t <T> control = x.template segment<nu>(nx);
        DynamicsModel::DynamicParams dyn_params = DynamicsModel::DynamicParams::Zero();
        DynamicsModel::output_t <T> output;

        DynamicsModel::state_t <T> state_dot;
        dynamicsModel.template dynamics<T>(state_dot, state, control, dyn_params, output);

        cost = state_dot.template dot(state_dot);
    }
};

/** create solver */
template<typename Problem>
class SQPSolver;
template<typename Problem>
class SQPSolver : public SQPBase<SQPSolver<Problem>, Problem>
{
public:
    using Base = SQPBase<SQPSolver<Problem>, Problem>;
    using typename Base::scalar_t;
    using typename Base::nlp_variable_t;
    using typename Base::nlp_hessian_t;
};

int main()
{
    const std::string modelparams_filepath = "/home/johannes/identification/eg4_xflr.yaml";

    /* Trim with casadi model */
    {
        using namespace flight_model::casadi_model::fixed_wing;
        FixedWingDynamics model(modelparams_filepath, flight_model::AttQuat,
                                {{"air_density", 1.15}}, {"v_wind_N", "v_wind_E", "v_wind_D"});

        using namespace casadi;
        DM vW = DM::vertcat({0, 0, 0});
        DM x0 = model.getDefaultInitialState();
        DM u0 = DM::vertcat({0, 0, 0, 0});
        model.trim(vW, DM::vertcat({x0, u0}),
                model.u_trim_bounds(Slice(), 0), model.u_trim_bounds(Slice(), 1),
                model.x_trim_bounds(Slice(), 0), model.x_trim_bounds(Slice(), 1));

        DM x_trim = model.getTrimState();
        DM u_trim = model.getTrimControl();
        std::cout << "x_trim: " << x_trim << "\n";
        std::cout << "u_trim: " << u_trim << "\n";

        const Function f = model.getNumericDynamics();
        std::cout << "x_dot: " << f(SXVector({x_trim, u_trim, {}, vW}))[0].nonzeros() << "\n";
    }


    /* Trim with eigen model */
    {
        using Solver = SQPSolver<TrimProblem>;
        Solver solver;

        /* Setup dynamics */
        solver.problem.dynamicsModel.set_state_representation(flight_model::LongitudinalEulerAoa);
//        solver.problem.dynamicsModel.load_params_from_yaml(modelparams_filepath);

        using namespace flight_model::eigen_model::fixed_wing::static_parameters;
        StaticParamsBuilder staticParamsBuilder;
        staticParamsBuilder[rho] = 1.15;
        solver.problem.dynamicsModel.set_static_params(staticParamsBuilder.vec());

        /* Bounds */
        solver.upper_bound_x()
                << solver.problem.dynamicsModel.x_trim_ubound, solver.problem.dynamicsModel.u_trim_ubound;
        solver.lower_bound_x()
                << solver.problem.dynamicsModel.x_trim_lbound, solver.problem.dynamicsModel.u_trim_lbound;

        /* Initial guess */
        DynamicsModel::State init_state = solver.problem.dynamicsModel.get_default_initial_state();
        DynamicsModel::Control init_control = {0, 0};
//        DynamicsModel::Control init_control = {0, 0, 0, 0};
        Solver::nlp_variable_t x0 = (Solver::nlp_variable_t() << init_state, init_control).finished();
        Solver::nlp_dual_t y0 = Solver::nlp_dual_t().setZero();

        /* Solve */
        solver.settings().max_iter = 100;
        solver.settings().line_search_max_iter = 5;
        solver.settings().eps_prim = 1e-8;
        solver.solve(x0, y0);

        Solver::nlp_variable_t x = solver.primal_solution();

        std::cout << "iter " << solver.info().iter << std::endl;
        std::cout << "Solution " << x.transpose() << std::endl;

        DynamicsModel::State x_trim = x.segment<TrimProblem::nx>(0);
        DynamicsModel::Control u_trim = x.segment<TrimProblem::nu>(TrimProblem::nx);
        std::cout << "\nx_trim: " << x_trim.transpose() << ", u_trim: " << u_trim.transpose() << "\n";
        {
            /* Evaluate Dynamics at trim point (should be zero) */
            DynamicsModel::DynamicParams dyn_params = DynamicsModel::DynamicParams::Zero();
            DynamicsModel::Output output;

            DynamicsModel::State dynamics;
            solver.problem.dynamicsModel.dynamics(dynamics, x_trim, u_trim, dyn_params, output);
            std::cout << "x_dot: " << dynamics.transpose() << "\n";
        }
        // x_trim = [12.5021, -0.0327415, 8.59739e-08, -0.0680528];
        // u_trim = [-0.0105831, 0];

        //    A = [
        //    -0.0562547     6.09266  -0.0446645    -9.79989;
        //    -0.125394    -9.34381    0.887762   0.0276905;
        //    0.000230966    -90.5807     -8.6571           0;
        //    0           0           1           0];
        //    B = [
        //    -0.190614   -0.423593;
        //    -0.478997 0.000664789;
        //    -73.532    0.111981;
        //    0           0];

        /** Linearize about trim point **/
        DynamicsModel::SystemMat A_trim;
        DynamicsModel::ControlMat B_trim;
        solver.problem.dynamicsModel.linearize_at(A_trim, B_trim, x_trim, u_trim);
        std::cout << "A:\n" << A_trim << "\n";
        std::cout << "B:\n" << B_trim << "\n";

        /* Compute LQR gain K */
        polymath::LinearSystem system_trim(A_trim, B_trim, {});

        Eigen::Matrix<double, DynamicsModel::nx, DynamicsModel::nx> Q;
        Eigen::Matrix<double, DynamicsModel::nu, DynamicsModel::nu> R;
        Eigen::Matrix<double, DynamicsModel::nx, DynamicsModel::nu> M;
        Q.setIdentity();
        R.setIdentity();
        M.setZero();

        std::cout << "\nLQR Setup" << "\n"
                  << "A_trim: \n" << A_trim << "\n"
                  << "B_trim: \n" << B_trim << "\n"
                  << "Q: \n" << Q << "\n"
                  << "R: \n" << R << "\n"
                  << "M: \n" << M << "\n";

        Eigen::MatrixXd K = polymath::oc::lqr(system_trim, Q, R, M);
        std::cout << "K:\n" << K << "\n";
    }
    return EXIT_SUCCESS;
}