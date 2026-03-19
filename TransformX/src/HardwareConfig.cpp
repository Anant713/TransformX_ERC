#include "HardwareConfig.hpp"

int servoPins[NUM_LEGS][NUM_JOINTS] = {
    {18, 19, 21},
    {12, 13, 14},
    {2,  3,  4},
    {22, 23, 25}
};

void initHardwareConfig()
{
    // Add hardware-specific init later if needed
}