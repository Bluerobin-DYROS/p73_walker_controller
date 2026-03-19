
#include <iostream>
#include "br_driver.hpp"

int main(void)
{
    ecat_shm_destroy();
    imu_shm_destroy();
    return 0;
}