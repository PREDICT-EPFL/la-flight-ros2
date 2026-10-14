#include "main_control_flight/fixedwing/FixedwingControl.hpp"

int main()
{
    using namespace control::flight::fixedwing;
    FixedwingControl fixedwingControl;
    fixedwingControl.test();

    return EXIT_SUCCESS;
}
