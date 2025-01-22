#include <stdio.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include "bno055.h"
//#include "bno055.c"

// Define the I2C address for the BNO055 sensor
#define BNO055_I2C_ADDR1 0x29

// I2C communication functions using Raspberry Pi's I2C
s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    int fd = wiringPiI2CSetup(dev_addr);  // Initialize I2C device with the given address
    if (fd == -1) {
        printf("Failed to initialize I2C device\n");
        return BNO055_ERROR;
    }

    // Write data to the register
    for (int i = 0; i < cnt; i++) {
        if (wiringPiI2CWriteReg8(fd, reg_addr + i, reg_data[i]) == -1) {
            printf("I2C write failed\n");
            return BNO055_ERROR;
        }
    }
    return BNO055_SUCCESS;
}

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    int fd = wiringPiI2CSetup(dev_addr);  // Initialize I2C device with the given address
    if (fd == -1) {
        printf("Failed to initialize I2C device\n");
        return BNO055_ERROR;
    }

    // Read data from the register
    for (int i = 0; i < cnt; i++) {
        reg_data[i] = wiringPiI2CReadReg8(fd, reg_addr + i);
        if (reg_data[i] == -1) {
            printf("I2C read failed\n");
            return BNO055_ERROR;
        }
    }
    return BNO055_SUCCESS;
}

void delay_msec(u32 msec) {
    usleep(msec * 1000);  // Convert milliseconds to microseconds
}

int main() {
    struct bno055_t bno055;
    struct bno055_euler_t euler_angles;

    // Initialize wiringPi for I2C setup
    if (wiringPiSetup() == -1) {
        printf("Failed to initialize wiringPi\n");
        return -1;
    }

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
