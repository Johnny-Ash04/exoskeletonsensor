#include <wiringPi.h>
#include <stdio.h>
#include "bno055.h"
//#include "bno055.c"

// I2C communication functions (These should be implemented according to your specific platform)
s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    // Implement I2C write function
    // ...
    return BNO055_SUCCESS; // Return 0 for success
}

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    // Implement I2C read function
    // ...
    return BNO055_SUCCESS; // Return 0 for success
}

void delay_msec(u32 msec) {
    // Implement delay function
    usleep(msec * 1000); // Convert milliseconds to microseconds
}

int main() {
    struct bno055_t bno055;
    struct bno055_euler_t euler_angles;

    // Configure I2C communication functions
    bno055.bus_write = I2C_bus_write;
    bno055.bus_read = I2C_bus_read;
    bno055.delay_msec = delay_msec;
    bno055.dev_addr = BNO055_I2C_ADDR1;  // Use the appropriate I2C address

    // Initialize BNO055
    if (bno055_init(&bno055) != BNO055_SUCCESS) {
        printf("BNO055 initialization failed!\n");
        return -1;
    }

    // Set the operation mode to NDOF (absolute orientation)
    if (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        printf("Failed to set operation mode!\n");
        return -1;
    }

    // Delay to allow sensor to switch modes
    delay_msec(BNO055_MODE_SWITCHING_DELAY);

    // Read Euler angles
    if (bno055_read_euler_hrp(&euler_angles) != BNO055_SUCCESS) {
        printf("Failed to read Euler angles!\n");
        return -1;
    }

    // Print Euler angles
    printf("Heading: %d\n", euler_angles.h);
    printf("Roll: %d\n", euler_angles.r);
    printf("Pitch: %d\n", euler_angles.p);

    return 0;
}
