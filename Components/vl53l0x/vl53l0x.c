/*
    Originnally from https://github.com/MarcelMG/VL53L0X-STM32F103
    For more copyright infomation,see ./LICENSE.txt
*/

// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include "stdio.h"
#include "VL53L0X.h"
#include "bsp_i2c.h"

/*
static HAL_StatusTypeDef i2c_read(uint8_t addr, uint8_t* data, uint8_t size){
    //return HAL_I2C_Master_Receive(&hi2c1, addr, data, size, 400);
    //return bsp_i2c_master_receive(&bsp_i2c_dev_vl53l0x, addr, data, size);
}

static HAL_StatusTypeDef i2c_write(uint8_t addr, uint8_t* data, uint8_t size){
    //return HAL_I2C_Master_Transmit(&hi2c1, addr, data, size, 400);
    //return bsp_i2c_master_send(&bsp_i2c_dev_vl53l0x, addr, data, size);
}
*/


// Record the current time to check an upcoming timeout against
//#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
//#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

#define sysTick_Time (HAL_GetTick())

void VL53L0X_setAddress(struct VL53L0X* dev, uint8_t new_addr){
    VL53L0X_writeReg(dev, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    dev->address = new_addr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53L0X_init(struct VL53L0X* dev){
    // VL53L0X_DataInit() begin

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (dev->io_2v8){
        VL53L0X_writeReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, VL53L0X_readReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
    }

    // "Set I2C standard mode"
    VL53L0X_writeReg(dev, 0x88, 0x00);

    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    dev->stop_variable = VL53L0X_readReg(dev, 0x91);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    VL53L0X_writeReg(dev, MSRC_CONFIG_CONTROL, VL53L0X_readReg(dev, MSRC_CONFIG_CONTROL) | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    VL53L0X_setSignalRateLimit(dev, 0.25);

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!VL53L0X_getSpadInfo(dev, &spad_count, &spad_type_is_aperture)){ return false; }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    VL53L0X_readMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    VL53L0X_writeReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++){
        if (i < first_spad_to_enable || spads_enabled == spad_count){
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1){
            spads_enabled++;
        }
    }

    VL53L0X_writeMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x09, 0x00);
    VL53L0X_writeReg(dev, 0x10, 0x00);
    VL53L0X_writeReg(dev, 0x11, 0x00);

    VL53L0X_writeReg(dev, 0x24, 0x01);
    VL53L0X_writeReg(dev, 0x25, 0xFF);
    VL53L0X_writeReg(dev, 0x75, 0x00);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x4E, 0x2C);
    VL53L0X_writeReg(dev, 0x48, 0x00);
    VL53L0X_writeReg(dev, 0x30, 0x20);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x30, 0x09);
    VL53L0X_writeReg(dev, 0x54, 0x00);
    VL53L0X_writeReg(dev, 0x31, 0x04);
    VL53L0X_writeReg(dev, 0x32, 0x03);
    VL53L0X_writeReg(dev, 0x40, 0x83);
    VL53L0X_writeReg(dev, 0x46, 0x25);
    VL53L0X_writeReg(dev, 0x60, 0x00);
    VL53L0X_writeReg(dev, 0x27, 0x00);
    VL53L0X_writeReg(dev, 0x50, 0x06);
    VL53L0X_writeReg(dev, 0x51, 0x00);
    VL53L0X_writeReg(dev, 0x52, 0x96);
    VL53L0X_writeReg(dev, 0x56, 0x08);
    VL53L0X_writeReg(dev, 0x57, 0x30);
    VL53L0X_writeReg(dev, 0x61, 0x00);
    VL53L0X_writeReg(dev, 0x62, 0x00);
    VL53L0X_writeReg(dev, 0x64, 0x00);
    VL53L0X_writeReg(dev, 0x65, 0x00);
    VL53L0X_writeReg(dev, 0x66, 0xA0);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x22, 0x32);
    VL53L0X_writeReg(dev, 0x47, 0x14);
    VL53L0X_writeReg(dev, 0x49, 0xFF);
    VL53L0X_writeReg(dev, 0x4A, 0x00);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x7A, 0x0A);
    VL53L0X_writeReg(dev, 0x7B, 0x00);
    VL53L0X_writeReg(dev, 0x78, 0x21);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x23, 0x34);
    VL53L0X_writeReg(dev, 0x42, 0x00);
    VL53L0X_writeReg(dev, 0x44, 0xFF);
    VL53L0X_writeReg(dev, 0x45, 0x26);
    VL53L0X_writeReg(dev, 0x46, 0x05);
    VL53L0X_writeReg(dev, 0x40, 0x40);
    VL53L0X_writeReg(dev, 0x0E, 0x06);
    VL53L0X_writeReg(dev, 0x20, 0x1A);
    VL53L0X_writeReg(dev, 0x43, 0x40);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x34, 0x03);
    VL53L0X_writeReg(dev, 0x35, 0x44);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x31, 0x04);
    VL53L0X_writeReg(dev, 0x4B, 0x09);
    VL53L0X_writeReg(dev, 0x4C, 0x05);
    VL53L0X_writeReg(dev, 0x4D, 0x04);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x44, 0x00);
    VL53L0X_writeReg(dev, 0x45, 0x20);
    VL53L0X_writeReg(dev, 0x47, 0x08);
    VL53L0X_writeReg(dev, 0x48, 0x28);
    VL53L0X_writeReg(dev, 0x67, 0x00);
    VL53L0X_writeReg(dev, 0x70, 0x04);
    VL53L0X_writeReg(dev, 0x71, 0x01);
    VL53L0X_writeReg(dev, 0x72, 0xFE);
    VL53L0X_writeReg(dev, 0x76, 0x00);
    VL53L0X_writeReg(dev, 0x77, 0x00);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x0D, 0x01);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0x01, 0xF8);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x8E, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    VL53L0X_writeReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_readReg(dev, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    dev->measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget(dev);

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!VL53L0X_performSingleRefCalibration(dev, 0x40)){ return false; }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!VL53L0X_performSingleRefCalibration(dev, 0x00)){ return false; }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    return true;
}

#include "bsp_i2c.h"
void VL53L0X_writeMulti(struct VL53L0X* dev, uint8_t reg, uint8_t* src, uint8_t count){
    
    dev->last_status = bsp_i2c_master_write_reg(&bsp_i2c_dev_vl53l0x, dev->address, reg, src, count);
    //HAL_I2C_Mem_Write(BSP_I2C_VL53L0X, dev->address, reg, I2C_MEMADD_SIZE_8BIT, src, count, 400);
    
    if(dev->last_status != HAL_OK){
        printf("vl53l0x write reg 0x%x failed, value:0x", reg);
        for(int i=0;i<count;i++){
            printf("%02x", src[i]);
        }
        printf("\r\n");
    }
    /*else{
        printf("vl53l0x write reg 0x%x success, value:0x", reg);
        for(int i=0;i<count;i++){
            printf("%02x", src[i]);
        }
        printf("\r\n");
    }*/

    
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void VL53L0X_readMulti(struct VL53L0X* dev, uint8_t reg, uint8_t* dst, uint8_t count){
    dev->last_status = bsp_i2c_master_read_reg(&bsp_i2c_dev_vl53l0x, dev->address, reg, dst, count);
    //HAL_I2C_Mem_Read(BSP_I2C_VL53L0X, dev->address, reg, I2C_MEMADD_SIZE_8BIT, dst, count, 400);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x read reg 0x%x failed, value:0x", reg);
        for(int i=0;i<count;i++){
            printf("%02x", dst[i]);
        }
        printf("\r\n");
    }/*else{
        printf("vl53l0x write reg 0x%x success, value:0x", reg);
        for(int i=0;i<count;i++){
            printf("%02x", dst[i]);
        }
        printf("\r\n");
    }*/
}

inline VL53L0X_writeReg(struct VL53L0X* dev, uint8_t reg, uint8_t value){
    VL53L0X_writeMulti(dev, reg, &value, 1);
}

void VL53L0X_writeReg16Bit(struct VL53L0X* dev, uint8_t reg, uint16_t value){
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    VL53L0X_writeMulti(dev, reg, &buf, 2);
}

void VL53L0X_writeReg32Bit(struct VL53L0X* dev, uint8_t reg, uint32_t value){
    uint8_t buf[4];
    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)(value & 0xFF);
    VL53L0X_writeMulti(dev, reg, &buf, 4);
}

inline uint8_t VL53L0X_readReg(struct VL53L0X* dev, uint8_t reg){
    uint8_t value;
    VL53L0X_readMulti(dev, reg, &value, 1);
    return value;
}

// Read a 16-bit register
uint16_t VL53L0X_readReg16Bit(struct VL53L0X* dev, uint8_t reg){
    uint16_t value;
    uint8_t buf[2];
    VL53L0X_readMulti(dev, reg, buf, 2);

    value = (uint16_t)(buf[0] << 8);
    value |= (uint16_t)buf[1];
    return value;
}

// Read a 32-bit register
uint32_t VL53L0X_readReg32Bit(struct VL53L0X* dev, uint8_t reg){
    uint32_t value;
    uint8_t buf[4];
    VL53L0X_readMulti(dev, reg, buf, 4);
    value = (uint32_t)(buf[0] << 24);
    value |= (uint32_t)(buf[1] << 16);
    value |= (uint32_t)(buf[2] << 8);
    value |= (uint32_t)buf[3];
    return value;
}

/*
// Write an 8-bit register
void VL53L0X_writeReg(struct VL53L0X* dev, uint8_t reg, uint8_t value){
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    dev->last_status = i2c_write(dev->address, buf, 2);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x write reg %x failed, data %x\r\n", reg, value);
    }
}

// Write a 16-bit register
void VL53L0X_writeReg16Bit(struct VL53L0X* dev, uint8_t reg, uint16_t value){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)(value >> 8);
    buf[2] = (uint8_t)(value & 0xFF);
    dev->last_status = i2c_write(dev->address, buf, 3);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x write reg %x failed, data %x\r\n", reg, value);
    }
}

// Write a 32-bit register
void VL53L0X_writeReg32Bit(struct VL53L0X* dev, uint8_t reg, uint32_t value){
    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = (uint8_t)(value >> 24);
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 8);
    buf[4] = (uint8_t)(value & 0xFF);
    dev->last_status = i2c_write(dev->address, buf, 5);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x write reg %lx failed, data %x\r\n", reg, value);
    }
}

// Read an 8-bit register
uint8_t VL53L0X_readReg(struct VL53L0X* dev, uint8_t reg){
    uint8_t value;
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_read(dev->address, &value, 1);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x read reg %x failed\r\n", reg);
    }
    return value;
}

// Read a 16-bit register
uint16_t VL53L0X_readReg16Bit(struct VL53L0X* dev, uint8_t reg){
    uint16_t value;
    uint8_t buf[2];
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_read(dev->address, buf, 2);
    value = (uint16_t)(buf[0] << 8);
    value |= (uint16_t)buf[1];
    if(dev->last_status != HAL_OK){
        printf("vl53l0x read reg %x failed\r\n", reg);
    }
    return value;
}

// Read a 32-bit register
uint32_t VL53L0X_readReg32Bit(struct VL53L0X* dev, uint8_t reg){
    uint32_t value;
    uint8_t buf[4];
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_read(dev->address, buf, 4);
    value = (uint32_t)(buf[0] << 24);
    value |= (uint32_t)(buf[1] << 16);
    value |= (uint32_t)(buf[2] << 8);
    value |= (uint32_t)buf[3];
    if(dev->last_status != HAL_OK){
        printf("vl53l0x read reg %x failed\r\n", reg);
    }
    return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void VL53L0X_writeMulti(struct VL53L0X* dev, uint8_t reg, uint8_t* src, uint8_t count){
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_write(dev->address, src, count);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x write reg %x failed\r\n", reg);
    }
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void VL53L0X_readMulti(struct VL53L0X* dev, uint8_t reg, uint8_t* dst, uint8_t count){
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_read(dev->address, dst, count);
    if(dev->last_status != HAL_OK){
        printf("vl53l0x read reg %x failed\r\n", reg);
    }
}

*/

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53L0X_setSignalRateLimit(struct VL53L0X* dev, float limit_Mcps){
    if (limit_Mcps < 0 || limit_Mcps > 511.99){ return false; }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    return true;
}

// Get the return signal rate limit check value in MCPS
float VL53L0X_getSignalRateLimit(struct VL53L0X* dev){
    return (float)VL53L0X_readReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X_setMeasurementTimingBudget(struct VL53L0X* dev, uint32_t budget_us){
    struct VL53L0X_SequenceStepEnables enables;
    struct VL53L0X_SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget){ return false; }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc){
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss){
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc){
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range){
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range){
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us){
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range){
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        dev->measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X_getMeasurementTimingBudget(struct VL53L0X* dev){
    struct VL53L0X_SequenceStepEnables enables;
    struct VL53L0X_SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc){
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss){
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc){
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range){
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range){
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool VL53L0X_setVcselPulsePeriod(struct VL53L0X* dev, enum VL53L0X_vcselPeriodType type, uint8_t period_pclks){
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    struct VL53L0X_SequenceStepEnables enables;
    struct VL53L0X_SequenceStepTimeouts timeouts;

    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."


    if (type == VcselPeriodPreRange){
        // "Set phase check limits"
        switch (period_pclks){
        case 12:
            VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;

        case 14:
            VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;

        case 16:
            VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;

        case 18:
            VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;

        default:
            // invalid period
            return false;
        }
        VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        VL53L0X_writeReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        VL53L0X_writeReg(dev, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    }
    else if (type == VcselPeriodFinalRange){
        switch (period_pclks){
        case 8:
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            VL53L0X_writeReg(dev, 0xFF, 0x01);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x30);
            VL53L0X_writeReg(dev, 0xFF, 0x00);
            break;

        case 10:
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            VL53L0X_writeReg(dev, 0xFF, 0x01);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
            VL53L0X_writeReg(dev, 0xFF, 0x00);
            break;

        case 12:
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            VL53L0X_writeReg(dev, 0xFF, 0x01);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
            VL53L0X_writeReg(dev, 0xFF, 0x00);
            break;

        case 14:
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            VL53L0X_writeReg(dev, 0xFF, 0x01);
            VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
            VL53L0X_writeReg(dev, 0xFF, 0x00);
            break;

        default:
            // invalid period
            return false;
        }

        // apply new VCSEL period
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range){
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(new_final_range_timeout_mclks));

        // set_sequence_step_timeout end
    }
    else{
        // invalid type
        return false;
    }

    // "Finally, the timing budget must be re-applied"

    VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = VL53L0X_readReg(dev, SYSTEM_SEQUENCE_CONFIG);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    VL53L0X_performSingleRefCalibration(dev, 0x0);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X_getVcselPulsePeriod(struct VL53L0X* dev, enum VL53L0X_vcselPeriodType type){
    if (type == VcselPeriodPreRange){
        return decodeVcselPeriod(VL53L0X_readReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else if (type == VcselPeriodFinalRange){
        return decodeVcselPeriod(VL53L0X_readReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else{ return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X_startContinuous(struct VL53L0X* dev, uint32_t period_ms){
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    if (period_ms != 0){
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        uint16_t osc_calibrate_val = VL53L0X_readReg16Bit(dev, OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0){
            period_ms *= osc_calibrate_val;
        }

        VL53L0X_writeReg32Bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        VL53L0X_writeReg(dev, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }
    else{
        // continuous back-to-back mode
        VL53L0X_writeReg(dev, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X_stopContinuous(struct VL53L0X* dev){
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, 0x00);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X_readRangeContinuousMillimeters(struct VL53L0X* dev){
    VL53L0X_startTimeout(dev);
    while ((VL53L0X_readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0){
        if (VL53L0X_checkTimeoutExpired(dev)){
            dev->did_timeout = true;
            return 65535;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    uint16_t range = VL53L0X_readReg16Bit(dev, RESULT_RANGE_STATUS + 10);

    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X_readRangeSingleMillimeters(struct VL53L0X* dev){
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    VL53L0X_startTimeout(dev);
    while (VL53L0X_readReg(dev, SYSRANGE_START) & 0x01){
        if (VL53L0X_checkTimeoutExpired(dev)){
            dev->did_timeout = true;
            return 65535;
        }
    }

    return VL53L0X_readRangeContinuousMillimeters(dev);
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L0X_timeoutOccurred(struct VL53L0X* dev){
    bool tmp = dev->did_timeout;
    dev->did_timeout = false;
    return tmp;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X_getSpadInfo(struct VL53L0X* dev, uint8_t* count, bool* type_is_aperture){
    uint8_t tmp;

    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);

    VL53L0X_writeReg(dev, 0xFF, 0x06);
    VL53L0X_writeReg(dev, 0x83, VL53L0X_readReg(dev, 0x83) | 0x04);
    VL53L0X_writeReg(dev, 0xFF, 0x07);
    VL53L0X_writeReg(dev, 0x81, 0x01);

    VL53L0X_writeReg(dev, 0x80, 0x01);

    VL53L0X_writeReg(dev, 0x94, 0x6b);
    VL53L0X_writeReg(dev, 0x83, 0x00);
    VL53L0X_startTimeout(dev);
    while (VL53L0X_readReg(dev, 0x83) == 0x00){
        if (VL53L0X_checkTimeoutExpired(dev)){ return false; }
    }
    VL53L0X_writeReg(dev, 0x83, 0x01);
    tmp = VL53L0X_readReg(dev, 0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    VL53L0X_writeReg(dev, 0x81, 0x00);
    VL53L0X_writeReg(dev, 0xFF, 0x06);
    VL53L0X_writeReg(dev, 0x83, VL53L0X_readReg(dev, 0x83) & ~0x04);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x01);

    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void VL53L0X_getSequenceStepEnables(struct VL53L0X* dev, struct VL53L0X_SequenceStepEnables* enables){
    uint8_t sequence_config = VL53L0X_readReg(dev, SYSTEM_SEQUENCE_CONFIG);

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53L0X_getSequenceStepTimeouts(struct VL53L0X* dev, struct VL53L0X_SequenceStepEnables* enables, struct VL53L0X_SequenceStepTimeouts* timeouts){
    timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(dev, VcselPeriodPreRange);

    timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(dev, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(dev, VcselPeriodFinalRange);

    timeouts->final_range_mclks = VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range){
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val){
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
        (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks){
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0){
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0){
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else{ return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks){
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks){
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X_performSingleRefCalibration(struct VL53L0X* dev, uint8_t vhv_init_byte){
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    VL53L0X_startTimeout(dev);
    while ((VL53L0X_readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0){
        if (VL53L0X_checkTimeoutExpired(dev)){ return false; }
    }

    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    VL53L0X_writeReg(dev, SYSRANGE_START, 0x00);

    return true;
}


void VL53L0X_startTimeout(struct VL53L0X* dev){
    dev->timeout_start_ms = sysTick_Time;
}

bool VL53L0X_checkTimeoutExpired(struct VL53L0X* dev){
    return (dev->io_timeout > 0 && (sysTick_Time - dev->timeout_start_ms) > dev->io_timeout);
}