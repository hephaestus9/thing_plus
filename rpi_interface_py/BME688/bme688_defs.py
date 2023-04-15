#/**
#* Copyright c 2021Bosch Sensortec GmbH. All rights reserved.
#*
#*BSD-3-Clause
#*
#* Redistribution and use in source andBinary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*
#* 1. Redistributions of source code must retain the above copyright
#*    notice, this list of conditions and the following disclaimer.
#*
#* 2. Redistributions inBinary form must reproduce the above copyright
#*    notice, this list of conditions and the following disclaimer in the
#*    documentation and/or other materials provided with the distribution.
#*
#* 3. Neither the name of the copyright holder nor the names of its
#*    contributors mayBe used to endorse or promote products derived from
#*    this software without specific prior written permission.
#*
#* THIS SOFTWARE IS PROVIDEDBY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,BUT NOT
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#* COPYRIGHT HOLDER OR CONTRIBUTORSBE LIABLE FOR ANY DIRECT, INDIRECT,
#* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#* INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#* SERVICES; LOSS OF USE, DATA, OR PROFITS; ORBUSINESS INTERRUPTION
#* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#* STRICT LIABILITY, OR TORT INCLUDING NEGLIGENCE OR OTHERWISE ARISING
#* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#* POSSIBILITY OF SUCH DAMAGE.
#*
#* @file      Bme68x_defs.h
#* @date       2021-05-24
#* @version    v4.4.6
#* 
#* modified    j.brian 4/11/2023
#*/


#/* PeriodBetween two polls value canBe givenBy user */
BME68X_PERIOD_POLL =                       10000

#/*BME68X unique chip identifier */
BME68X_CHIP_ID =                           0x61

#/* Period for a soft reset */
BME68X_PERIOD_RESET =                      10000

#/*BME68X lower I2C address */
BME68X_I2C_ADDR_LOW  =                     0x76

#/*BME68X higher I2C address */
BME68X_I2C_ADDR_HIGH  =                    0x77

#/* Soft reset command */
BME68X_SOFT_RESET_CMD   =                  0xb6

#/* Return code definitions */
#/* Success */
BME68X_OK  =                               0

#/* Errors */
#/* Null pointer passed */
BME68X_E_NULL_PTR =                        -1

#/* Communication failure */
BME68X_E_COM_FAIL =                        -2

#/* Sensor not found */
BME68X_E_DEV_NOT_FOUND =                   -3

#/* Incorrect length parameter */
BME68X_E_INVALID_LENGTH =                  -4

#/* Self test fail error */
BME68X_E_SELF_TEST =                       -5

#/* Warnings */
#/* Define a valid operation mode */
BME68X_W_DEFINE_OP_MODE =                  1

#/* No new data was found */
BME68X_W_NO_NEW_DATA  =                    2

#/* Define the shared heating duration */
BME68X_W_DEFINE_SHD_HEATR_DUR =            3

#/* Information - only available viaBme68x_dev.info_msg */
BME68X_I_PARAM_CORR =                      1

#/* Register map addresses in I2C */
#/* Register for 3rd group of coefficients */
BME68X_REG_COEFF3  =                       0x00

#/* 0th Field address*/
BME68X_REG_FIELD0  =                       0x1d

#/* 0th Current DAC address*/
BME68X_REG_IDAC_HEAT0  =                   0x50

#/* 0th Res heat address */
BME68X_REG_RES_HEAT0    =                  0x5a

#/* 0th Gas wait address */
BME68X_REG_GAS_WAIT0  =                    0x64

#/* Shared heating duration address */
BME68X_REG_SHD_HEATR_DUR  =                0x6E

#/* CTRL_GAS_0 address */
BME68X_REG_CTRL_GAS_0  =                   0x70

#/* CTRL_GAS_1 address */
BME68X_REG_CTRL_GAS_1  =                   0x71

#/* CTRL_HUM address */
BME68X_REG_CTRL_HUM   =                    0x72

#/* CTRL_MEAS address */
BME68X_REG_CTRL_MEAS  =                    0x74

#/* CONFIG address */
BME68X_REG_CONFIG    =                     0x75

#/* MEM_PAGE address */
BME68X_REG_MEM_PAGE  =                     0xf3

#/* Unique ID address */
BME68X_REG_UNIQUE_ID =                     0x83

#/* Register for 1st group of coefficients */
BME68X_REG_COEFF1  =                       0x8a

#/* Chip ID address */
BME68X_REG_CHIP_ID  =                      0xd0

#/* Soft reset address */
BME68X_REG_SOFT_RESET  =                   0xe0

#/* Register for 2nd group of coefficients */
BME68X_REG_COEFF2  =                       0xe1

#/* Variant ID Register */
BME68X_REG_VARIANT_ID =                    0xF0

#/* Enable/Disable macros */

#/* Enable */
BME68X_ENABLE   =                          0x01

#/* Disable */
BME68X_DISABLE   =                         0x00

#/* Variant ID macros */

#/* Low Gas variant */
BME68X_VARIANT_GAS_LOW  =                  0x00

#/* High Gas variant */
BME68X_VARIANT_GAS_HIGH  =                 0x01

#/* Oversampling setting macros */

#/* Switch off measurement */
BME68X_OS_NONE  =                          0

#/* Perform 1 measurement */
BME68X_OS_1X   =                           1

#/* Perform 2 measurements */
BME68X_OS_2X       =                       2

#/* Perform 4 measurements */
BME68X_OS_4X      =                        3

#/* Perform 8 measurements */
BME68X_OS_8X       =                       4

#/* Perform 16 measurements */
BME68X_OS_16X     =                        5

#/* IIR Filter settings */

#/* Switch off the filter */
BME68X_FILTER_OFF     =                    0

#/* Filter coefficient of 2 */
BME68X_FILTER_SIZE_1  =                    1

#/* Filter coefficient of 4 */
BME68X_FILTER_SIZE_3   =                   2

#/* Filter coefficient of 8 */
BME68X_FILTER_SIZE_7  =                    3

#/* Filter coefficient of 16 */
BME68X_FILTER_SIZE_15   =                  4

#/* Filter coefficient of 32 */
BME68X_FILTER_SIZE_31  =                   5

#/* Filter coefficient of 64 */
BME68X_FILTER_SIZE_63 =                    6

#/* Filter coefficient of 128 */
BME68X_FILTER_SIZE_127  =                  7

#/* ODR/Standby time macros */

#/* Standby time of 0.59ms */
BME68X_ODR_0_59_MS   =                     0

#/* Standby time of 62.5ms */
BME68X_ODR_62_5_MS =                       1

#/* Standby time of 125ms */
BME68X_ODR_125_MS  =                       2

#/* Standby time of 250ms */
BME68X_ODR_250_MS   =                      3

#/* Standby time of 500ms */
BME68X_ODR_500_MS  =                       4

#/* Standby time of 1s */
BME68X_ODR_1000_MS  =                      5

#/* Standby time of 10ms */
BME68X_ODR_10_MS  =                        6

#/* Standby time of 20ms */
BME68X_ODR_20_MS =                         7

#/* No standby time */
BME68X_ODR_NONE =                          8

#/* Operating mode macros */

#/* Sleep operation mode */
BME68X_SLEEP_MODE  =                       0

#/* Forced operation mode */
BME68X_FORCED_MODE   =                     1

#/* Parallel operation mode */
BME68X_PARALLEL_MODE  =                    2

#/* Sequential operation mode */
BME68X_SEQUENTIAL_MODE  =                  3

#/* SPI page macros */

#/* SPI memory page 0 */
BME68X_MEM_PAGE0 =                         0x10

#/* SPI memory page 1 */
BME68X_MEM_PAGE1  =                        0x00

#/* Coefficient index macros */

#/* Length for all coefficients */
BME68X_LEN_COEFF_ALL  =                    42

#/* Length for 1st group of coefficients */
BME68X_LEN_COEFF1  =                       23

#/* Length for 2nd group of coefficients */
BME68X_LEN_COEFF2   =                      14

#/* Length for 3rd group of coefficients */
BME68X_LEN_COEFF3  =                       5

#/* Length of the field */
BME68X_LEN_FIELD  =                        17

#/* LengthBetween two fields */
BME68X_LEN_FIELD_OFFSET  =                 17

#/* Length of the configuration register */
BME68X_LEN_CONFIG   =                      5

#/* Length of the interleavedBuffer */
BME68X_LEN_INTERLEAVE_BUFF  =              20

#/* Coefficient index macros */

#/* Coefficient T2 LSB position */
BME68X_IDX_T2_LSB  =                       0

#/* Coefficient T2 MSB position */
BME68X_IDX_T2_MSB  =                       1

#/* Coefficient T3 position */
BME68X_IDX_T3  =                           2

#/* Coefficient P1 LSB position */
BME68X_IDX_P1_LSB  =                       4

#/* Coefficient P1 MSB position */
BME68X_IDX_P1_MSB  =                       5

#/* Coefficient P2 LSB position */
BME68X_IDX_P2_LSB    =                     6

#/* Coefficient P2 MSB position */
BME68X_IDX_P2_MSB    =                     7

#/* Coefficient P3 position */
BME68X_IDX_P3     =                        8

#/* Coefficient P4 LSB position */
BME68X_IDX_P4_LSB   =                      10

#/* Coefficient P4 MSB position */
BME68X_IDX_P4_MSB  =                       11

#/* Coefficient P5 LSB position */
BME68X_IDX_P5_LSB  =                       12

#/* Coefficient P5 MSB position */
BME68X_IDX_P5_MSB  =                       13

#/* Coefficient P7 position */
BME68X_IDX_P7    =                         14

#/* Coefficient P6 position */
BME68X_IDX_P6   =                          15

#/* Coefficient P8 LSB position */
BME68X_IDX_P8_LSB  =                       18

#/* Coefficient P8 MSB position */
BME68X_IDX_P8_MSB     =                    19

#/* Coefficient P9 LSB position */
BME68X_IDX_P9_LSB    =                     20

#/* Coefficient P9 MSB position */
BME68X_IDX_P9_MSB  =                       21

#/* Coefficient P10 position */
BME68X_IDX_P10     =                       22

#/* Coefficient H2 MSB position */
BME68X_IDX_H2_MSB   =                      23

#/* Coefficient H2 LSB position */
BME68X_IDX_H2_LSB    =                     24

#/* Coefficient H1 LSB position */
BME68X_IDX_H1_LSB   =                      24

#/* Coefficient H1 MSB position */
BME68X_IDX_H1_MSB  =                       25

#/* Coefficient H3 position */
BME68X_IDX_H3    =                         26

#/* Coefficient H4 position */
BME68X_IDX_H4     =                        27

#/* Coefficient H5 position */
BME68X_IDX_H5   =                          28

#/* Coefficient H6 position */
BME68X_IDX_H6  =                           29

#/* Coefficient H7 position */
BME68X_IDX_H7   =                          30

#/* Coefficient T1 LSB position */
BME68X_IDX_T1_LSB  =                       31

#/* Coefficient T1 MSB position */
BME68X_IDX_T1_MSB  =                       32

#/* Coefficient GH2 LSB position */
BME68X_IDX_GH2_LSB   =                     33

#/* Coefficient GH2 MSB position */
BME68X_IDX_GH2_MSB   =                     34

#/* Coefficient GH1 position */
BME68X_IDX_GH1       =                     35

#/* Coefficient GH3 position */
BME68X_IDX_GH3    =                        36

#/* Coefficient res heat value position */
BME68X_IDX_RES_HEAT_VAL   =                37

#/* Coefficient res heat range position */
BME68X_IDX_RES_HEAT_RANGE  =               39

#/* Coefficient range switching error position */
BME68X_IDX_RANGE_SW_ERR  =                 41

#/* Gas measurement macros */

#/* Disable gas measurement */
BME68X_DISABLE_GAS_MEAS   =                0x00

#/* Enable gas measurement low */
BME68X_ENABLE_GAS_MEAS_L   =               0x01

#/* Enable gas measurement high */
BME68X_ENABLE_GAS_MEAS_H  =                0x02

#/* Heater control macros */

#/* Enable heater */
BME68X_ENABLE_HEATER   =                   0x00

#/* Disable heater */
BME68X_DISABLE_HEATER  =                   0x01

#/* 0 degree Celsius */
BME68X_MIN_TEMPERATURE =                   0

#/* 60 degree Celsius */
BME68X_MAX_TEMPERATURE =                   6000

#/* 900 hecto Pascals */
BME68X_MIN_PRESSURE  =                     90000

#/* 1100 hecto Pascals */
BME68X_MAX_PRESSURE =                      110000

#/* 20% relative humidity */
BME68X_MIN_HUMIDITY  =                     20000

#/* 80% relative humidity*/
BME68X_MAX_HUMIDITY  =                     80000


BME68X_HEATR_DUR1      =                   1000
BME68X_HEATR_DUR2      =                   2000
BME68X_HEATR_DUR1_DELAY   =                1000000
BME68X_HEATR_DUR2_DELAY   =                2000000
BME68X_N_MEAS          =                   6
BME68X_LOW_TEMP       =                    150
BME68X_HIGH_TEMP      =                    350

#/* Mask macros */
#/* Mask for number of conversions */
BME68X_NBCONV_MSK    =                     0X0f

#/* Mask for IIR filter */
BME68X_FILTER_MSK  =                      0X1c

#/* Mask for ODR[3] */
BME68X_ODR3_MSK    =                       0x80

#/* Mask for ODR[2:0] */
BME68X_ODR20_MSK   =                       0xe0

#/* Mask for temperature oversampling */
BME68X_OST_MSK     =                       0Xe0

#/* Mask for pressure oversampling */
BME68X_OSP_MSK     =                       0X1c

#/* Mask for humidity oversampling */
BME68X_OSH_MSK     =                       0X07

#/* Mask for heater control */
BME68X_HCTRL_MSK   =                       0x08

#/* Mask for run gas */
BME68X_RUN_GAS_MSK  =                      0x30

#/* Mask for operation mode */
BME68X_MODE_MSK     =                      0x03

#/* Mask for res heat range */
BME68X_RHRANGE_MSK     =                   0x30

#/* Mask for range switching error */
BME68X_RSERROR_MSK   =                     0xf0

#/* Mask for new data */
BME68X_NEW_DATA_MSK =                      0x80

#/* Mask for gas index */
BME68X_GAS_INDEX_MSK  =                    0x0f

#/* Mask for gas range */
BME68X_GAS_RANGE_MSK =                     0x0f

#/* Mask for gas measurement valid */
BME68X_GASM_VALID_MSK   =                  0x20

#/* Mask for heater stability */
BME68X_HEAT_STAB_MSK  =                    0x10

#/* Mask for SPI memory page */
BME68X_MEM_PAGE_MSK  =                     0x10

#/* Mask for reading a register in SPI */
BME68X_SPI_RD_MSK    =                     0x80

#/* Mask for writing a register in SPI */
BME68X_SPI_WR_MSK    =                     0x7f

#/* Mask for the H1 calibration coefficient */
BME68X_BIT_H1_DATA_MSK  =                  0x0f

#/* Position macros */

#/* FilterBit position */
BME68X_FILTER_POS    =                     2

#/* Temperature oversamplingBit position */
BME68X_OST_POS       =                     5

#/* Pressure oversamplingBit position */
BME68X_OSP_POS     =                       2

#/* ODR[3]Bit position */
BME68X_ODR3_POS   =                        7

#/* ODR[2:0]Bit position */
BME68X_ODR20_POS   =                       5

#/* Run gasBit position */
BME68X_RUN_GAS_POS  =                      4

#/* Heater controlBit position */
BME68X_HCTRL_POS    =                      3

bit_vars = None

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

#/* Macro to combine two 8Bit data's to form a 16Bit data */
def BME68X_CONCAT_BYTES(msb, lsb, signed=True): 
    val = msb << 8 | lsb
    if signed:
        val = twos_comp(val, 16)
    return val

#/* Macro to setBits */
def BME68X_SET_BITS(reg_data, bitname, data):
    global bit_vars
    val = ((reg_data & ~(bit_vars[bitname + "_MSK"])) | ((data << bit_vars[bitname + "_POS"]) & bit_vars[bitname + "_MSK"]))
    return val

#/* Macro to getBits */
def BME68X_GET_BITS(reg_data, bitname):
   global bit_vars
   val = reg_data & bit_vars[bitname + "_MSK"] >> bit_vars[bitname + "_POS"]
   return val

#/* Macro to setBits starting from position 0 */
def BME68X_SET_BITS_POS_0(reg_data, bitname, data):
    global bit_vars
    val = reg_data & ~bit_vars[bitname + "_MSK"] | data & bit_vars[bitname + "_MSK"]
    return val

#/* Macro to getBits starting from position 0 */
def BME68X_GET_BITS_POS_0(reg_data, bitname):
   global bit_vars
   val = reg_data & bit_vars[bitname + "_MSK"]
   return val


#/**
# *BME68X_INTF_RET_SUCCESS is the success return value read/write interface return type which canBe
# * overwrittenBy theBuild system. The default is set to 0. It is used to check for a successful
# * execution of the read/write functions
# */
BME68X_INTF_RET_SUCCESS =                  0


#/********************************************************* */
#/*!               Function Pointers                       */
#/********************************************************* */

#/*!
# * @briefBus communication function pointer which shouldBe mapped to
# * the platform specific read functions of the user
# *
# * @param[in]     reg_addr : 8bit register address of the sensor
# * @param[out]    reg_data : Data from the specified address
# * @param[in]     length   : Length of the reg_data array
# * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
# *                           for interface related callbacks
# * @retval 0 for Success
# * @retval Non-zero for Failure
# */
#typedefBME68X_INTF_RET_TYPE *bme68x_read_fptr_t uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr;

#/*!
# * @briefBus communication function pointer which shouldBe mapped to
# * the platform specific write functions of the user
# *
# * @param[in]     reg_addr : 8bit register address of the sensor
# * @param[out]    reg_data : Data to the specified address
# * @param[in]     length   : Length of the reg_data array
# * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
# *                           for interface related callbacks
# * @retval 0 for Success
# * @retval Non-zero for Failure
# *
# */
#typedefBME68X_INTF_RET_TYPE *bme68x_write_fptr_t uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr;

#/*!
# * @brief Delay function pointer which shouldBe mapped to
# * delay function of the user
# *
# * @param period - The time period in microseconds
# * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
# *                           for interface related callbacks
# */
#typedef void *bme68x_delay_us_fptr_tuint32_t period, void *intf_ptr;

#/*
# * @brief Generic communication function pointer
# * @param[in] dev_id: Place holder to store the id of the device structure
# *                    CanBe used to store the index of the Chip select or
# *                    I2C address of the device.
# * @param[in] reg_addr: Used to select the register the where data needs to
# *                     Be read from or written to.
# * @param[in,out] reg_data: Data array to read/write
# * @param[in] len: Length of the data array
# */

#/*
# * @brief Interface selection Enumerations
# */
bme68x_intf = {
#    /*! SPI interface */
   "BME68X_SPI_INTF": False,
#    /*! I2C interface */
   "BME68X_I2C_INTF": True
}

#/* Structure definitions */

#/*
# * @brief Sensor field data structure
# */
bme68x_data = {
#    /*! Contains new_data, gasm_valid & heat_stab */
    "status": "",
#
#    /*! The index of the heater profile used */
    "gas_index": "",
#
#    /*! Measurement index to track order */
    "meas_index": "",
#
#    /*! Heater resistance */
    "res_heat": "",
#
#    /*! Current DAC */
    "idac": "",
#
#    /*! Gas wait period */
    "gas_wait": "",
#
#    /*! Temperature in degree celsius x100 */
    "temperature": "",
#
#    /*! Pressure in Pascal */
    "pressure": "",
#
#    /*! Humidity in % relative humidity x1000 */
    "humidity": "",
#
#    /*! Gas resistance in Ohms */
    "gas_resistance": ""
#
}

#/*
# * @brief Structure to hold the calibration coefficients
# */
bme68x_calib_data = {
#    /*! Calibration coefficient for the humidity sensor */
    "par_h1": "",
#
#    /*! Calibration coefficient for the humidity sensor */
    "par_h2": "",
#
#    /*! Calibration coefficient for the humidity sensor */
    "par_h3": "",
#
#    /*! Calibration coefficient for the humidity sensor */
    "par_h4": "",
#
#    /*! Calibration coefficient for the humidity sensor */
    "par_h5": "",
#
#    /*! Calibration coefficient for the humidity sensor */
    "par_h6": "",
#
#    /*! Calibration coefficient for the humidity sensor */
    "par_h7": "",
#
#    /*! Calibration coefficient for the gas sensor */
    "par_gh1": "",
#
#    /*! Calibration coefficient for the gas sensor */
    "par_gh2": "",
#
#    /*! Calibration coefficient for the gas sensor */
    "par_gh3": "",
#
#    /*! Calibration coefficient for the temperature sensor */
    "par_t1": "",
#
#    /*! Calibration coefficient for the temperature sensor */
    "par_t2": "",
#
#    /*! Calibration coefficient for the temperature sensor */
    "par_t3": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p1": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p2": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p3": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p4": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p5": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p6": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p7": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p8": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p9": "",
#
#    /*! Calibration coefficient for the pressure sensor */
    "par_p10": "",
#
#    /*! Variable to store the intermediate temperature coefficient */
    "t_fine": "",
#
#    /*! Heater resistance range coefficient */
    "res_heat_range": "",
#
#    /*! Heater resistance value coefficient */
    "res_heat_val": "",
#
#    /*! Gas resistance range switching error coefficient */
    "range_sw_err": ""
}

#/*
# * @briefBME68X sensor settings structure which comprises of ODR,
# * over-sampling and filter settings.
# */
bme68x_conf = {
#    /*! Humidity oversampling. Refer @ref osx*/
    "os_hum": 0,
#
#    /*! Temperature oversampling. Refer @ref osx */
    "os_temp": 0,
#
#    /*! Pressure oversampling. Refer @ref osx */
    "os_pres": 0,
#
#    /*! Filter coefficient. Refer @ref filter*/
    "filter": 0,
#
#    /*!
#     * Standby timeBetween sequential mode measurement profiles.
#     * Refer @ref odr
#     */
    "odr": 0
}

#/*
# * @briefBME68X gas heater configuration
# */
bme68x_heatr_conf = {
#    /*! Enable gas measurement. Refer @ref en_dis */
    "enable": "",
#
#    /*! Store the heater temperature for forced mode degree Celsius */
    "heatr_temp": "",
#
#    /*! Store the heating duration for forced mode in milliseconds */
    "heatr_dur": "",
#
#    /*! Store the heater temperature profile in degree Celsius */
    "heatr_temp_prof": "",
#
#    /*! Store the heating duration profile in milliseconds */
    "heatr_dur_prof": "",
#
#    /*! Variable to store the length of the heating profile */
    "profile_len": "",
#
#    /*!
#     * Variable to store heating duration for parallel mode
#     * in milliseconds
#     */
    "shared_heatr_dur": ""
}

#/*
# * @briefBME68X device structure
# */
bme68x_dev = {

    "chip_addr": BME68X_I2C_ADDR_HIGH,

#    /*! Chip Id */
    "chip_id": "",
#
#    /*!
#     * The interface pointer is used to enable the user
#     * to link their interface descriptors for reference during the
#     * implementation of the read and write interfaces to the
#     * hardware.
#     */
    "intf_ptr": "",

#    /*!
#     *             Variant id
#     * ----------------------------------------
#     *     Value   |           Variant
#     * ----------------------------------------
#     *      0      |  BME68X_VARIANT_GAS_LOW
#     *      1      |  BME68X_VARIANT_GAS_HIGH
#     * ----------------------------------------
#     */
    "variant_id": "",
#
#    /*! Ambient temperature in Degree C*/
    "amb_temp": "",
#
#    /*! Sensor calibration data */
    "calib": bme68x_calib_data,
#
#    /*! Read function pointer */
    "read": "",
#
#    /*! Write function pointer */
    "write": "",
#
#    /*! Delay function pointer */
    "delay_us": "",
#
#    /*! To store interface pointer error */
   "intf_rslt": "",

#    /*! Store the info messages */
    "info_msg": "",

    "op_mode": "",

    "device": "",

    "set_device": ""
}

bit_vars = vars()