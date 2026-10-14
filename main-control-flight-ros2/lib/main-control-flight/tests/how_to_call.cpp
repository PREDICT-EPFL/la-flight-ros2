#include "awe_control/AweHeliControl.hpp"

class ActiveParser
{
public:
    enum ActivationMode { TAP, HOLD } activation_mode{HOLD};

    void set_activation_mode(ActivationMode activationMode)
    {
        switch (activationMode)
        {
            case TAP:
            case HOLD:
                // OK
                break;
            default:
                activationMode = HOLD;
                std::cout << "ActiveParser: Unknown activation mode. Using HOLD as default\n";
        }
        activation_mode = activationMode;
    }

    enum Compare
    {
        GREATER, SMALLER, EQUAL
    };
    template<typename T = bool>
    bool wants_active_hold(T active_switch, Compare comparison_operator, const T &value)
    {
        bool wants_active = press(active_switch, comparison_operator, value);
        wanted_active = wants_active;
        return wants_active;
    }
    template<typename T = bool>
    bool wants_active_tap(T active_switch, Compare comparison_operator, const T &value)
    {
        bool wants_active = press(active_switch, comparison_operator, value);
        bool return_value = (wants_active && !wanted_active);
        wanted_active = wants_active;
        return return_value;
    }

    template<typename T = bool>
    bool wants_active(T active_switch, Compare comparison_operator, const T &value)
    {
        if (activation_mode == TAP)
        {
            return wants_active_tap<T>(active_switch, comparison_operator, value);
        }
        else
        {
            return wants_active_hold<T>(active_switch, comparison_operator, value);
        }
    }

private:
    bool wanted_active{false};

    template<typename T = bool>
    bool press(T active_switch, Compare comparison_operator, const T &value)
    {
        bool wants_active{false};

        /* User condition for active */
        switch (comparison_operator)
        {
            case GREATER:
                wants_active = (active_switch > value);
                break;
            case SMALLER:
                wants_active = (active_switch < value);
                break;
            case EQUAL:
                wants_active = (active_switch == value);
                break;
            default:
                wants_active = false;
        }
        return wants_active;
    }
};

ActiveParser m_param_parser_up;
ActiveParser m_param_parser_dn;
ActiveParser m_param_parser_right;
ActiveParser m_param_parser_left;

awe_control3::AweHeliController m_awe_heli_control;

void control_lift()
{
    /* Parameter triggers */
    bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
    bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
    bool p_right = (m_param_parser_right.wants_active_tap(manual.param_right_left, ActiveParser::GREATER, 0.5));
    bool p_left = (m_param_parser_left.wants_active_tap(manual.param_right_left, ActiveParser::SMALLER, -0.5));

    constexpr double normlift_step = 0.02;
    if (p_up || p_dn)
    {
        int param_manip{0};
        if (p_up) { param_manip = 1; }
        if (p_dn) { param_manip = -1; }

        m_awe_heli_control.p_accdir_trim(0) += param_manip * normlift_step;
        std::cout << "d_liftdir_trim: " << m_awe_heli_control.p_accdir_trim.transpose() << " ("
                  << (int) degrees(std::atan2(m_awe_heli_control.p_accdir_trim(1), m_awe_heli_control.p_accdir_trim(0)))
                  << " @ " << m_awe_heli_control.p_accdir_trim.norm() << ")\n";

    }
    if (p_right || p_left)
    {
        int param_manip{0};
        if (p_right) { param_manip = 1; }
        if (p_left) { param_manip = -1; }

        m_awe_heli_control.p_accdir_trim(1) += param_manip * normlift_step;
        std::cout << "d_liftdir_trim: " << m_awe_heli_control.p_accdir_trim.transpose() << " ("
                  << (int) degrees(std::atan2(m_awe_heli_control.p_accdir_trim(1), m_awe_heli_control.p_accdir_trim(0)))
                  << " @ " << m_awe_heli_control.p_accdir_trim.norm() << ")\n";
    }

    const double collective = -manual.thrust;
    const Eigen::Vector2d normalized_lifts = m_awe_heli_control.get_normlifts(states[0].pos, states[1].pos,
                                                                              collective,
                                                                              {manual.pitch, -manual.roll},
                                                                              m_awe_heli_control.p_accdir_trim);
    const Eigen::Vector2d elev_cmds = -0.05 * normalized_lifts; // Negative elevator command generates more lift

    commands[0].elevator = elev_cmds(0);
    commands[1].elevator = elev_cmds(1);
}