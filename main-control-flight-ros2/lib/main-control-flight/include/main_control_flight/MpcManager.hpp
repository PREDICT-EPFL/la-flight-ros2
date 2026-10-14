#ifndef SRC_MPCMANAGER_HPP
#define SRC_MPCMANAGER_HPP

#include "main_control_common/ControlTypes.hpp"
#include "main_control_flight/fixedwing/FixedwingTypes.hpp"

namespace control {
namespace flight {

using namespace control_common::types;

class MpcInterface
{
public:
    TimePoint mpc_command_stamp = SteadyClock::now();
    fixedwing_types::FixedwingCommand command{};
    bool mpc_armed{false};
    bool mpc_wasAvail{false};
    bool mpc_active{false};
    bool mpc_wasActive{false};

    bool evaluate_mpc_active(bool main_control_active)
    {
        mpc_wasInhibit = mpc_inhibit;
        mpc_wasActive = mpc_active;

        const unsigned mpc_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                SteadyClock::now() - mpc_command_stamp).count();
        bool mpc_avail = (mpc_age_ms < timeout_ms);

        if (mpc_avail)
        {
            if (!mpc_wasAvail && (!mpc_armed || mpc_inhibit)) { std::cout << "MPC avail\n"; }

            if (mpc_inhibit)
            {
                /* Check condition for disabling inhibit */
                const unsigned mpc_inhibit_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        SteadyClock::now() - mpc_inhibit_stamp).count();

                if (mpc_inhibit_age_ms > inh_rehab_ms) // Disable inhibit after 5s (and MPC avail)
                {
                    mpc_inhibit = false;
                    //if (mpc_wasInhibit) { std::cout << "MPC not inhibit\n"; }
                }
            }

            /* Evaluate MPC active */
            mpc_active = (main_control_active && !mpc_inhibit && mpc_armed);
        }
        else
        {
            if (mpc_wasAvail && !mpc_armed) { std::cout << "MPC not avail (" << mpc_age_ms << " ms since last)\n"; }

            mpc_active = false;

            /* Inhibit MPC for a while */
            mpc_inhibit = true;
            mpc_inhibit_stamp = SteadyClock::now();
        }

        if (mpc_active && !mpc_wasActive) { std::cout << "MPC active\n"; }

        if (!mpc_active && mpc_wasActive) { std::cout << "MPC not active (" << mpc_age_ms << " ms since last)\n"; }
        if (mpc_inhibit && !mpc_wasInhibit) { std::cout << "MPC inhibit\n"; }

//    std::cout << "DEBUG MPC: avail " << mpc_avail << " (" << mpc_wasAvail << "), "
//              << "inh " << mpc_inhibit << " (" << mpc_wasInhibit << "), "
//              << "act " << mpc_active << " (" << mpc_wasActive << ")" << "\n";

        mpc_wasAvail = mpc_avail;
        return mpc_active;
    }

    TimePoint mpc_inhibit_stamp = SteadyClock::now();
    bool mpc_inhibit{false};
    bool mpc_wasInhibit{false};

    unsigned timeout_ms{60};
    unsigned inh_rehab_ms{5000};
};

}
}

#endif //SRC_MPCMANAGER_HPP
