#/*!
# *  @file Adafruit_MAX1704X.h
# *
# * 	I2C Driver for the Adafruit MAX17048 Battery Monitor
# *
# * 	This is a library for the Adafruit MAX17048 breakout:
# * 	https://www.adafruit.com/products/5580
# *
# * 	Adafruit invests time and resources providing this open source code,
# *  please support Adafruit and open-source hardware by purchasing products from
# * 	Adafruit!
# *
# *
# *	BSD license (see license.txt)
# */
#/*!
#*  @file Adafruit_MAX1704X.cpp
#*
#*  @mainpage Adafruit MAX17048 Battery Monitor library
#*
#*  @section intro_sec Introduction
#*
#* 	I2C Driver for the Adafruit MAX17048 Battery Monitor library
#*
#* 	This is a library for the Adafruit MAX17048 breakout:
#* 	https://www.adafruit.com/product/5580
#*
#* 	Adafruit invests time and resources providing this open source code,
#*  please support Adafruit and open-source hardware by purchasing products from
#* 	Adafruit!
#*
#*  @section dependencies Dependencies
#*  This library depends on the Adafruit BusIO library
#*
#*  @section author Author
#*
#*  Limor Fried (Adafruit Industries)
#*
#* 	@section license License
#*
#* 	BSD (see license.txt)
#*
#* 	@section  HISTORY
#*
#*     v1.0 - First release
#*/

import time

MAX17048_I2CADDR_DEFAULT = 0x36 #///< MAX17048 default i2c address

MAX1704X_VCELL_REG   = 0x02  #///< Register that holds cell voltage
MAX1704X_SOC_REG     = 0x04  #///< Register that holds cell state of charge
MAX1704X_MODE_REG    = 0x06  #///< Register that manages mode
MAX1704X_VERSION_REG = 0x08  #///< Register that has IC version
MAX1704X_HIBRT_REG =   0x0A  #///< Register that manages hibernation
MAX1704X_CONFIG_REG =  0x0C  #///< Register that manages configuration
MAX1704X_VALERT_REG =  0x14  #///< Register that holds voltage alert values
MAX1704X_CRATE_REG =   0x16  #///< Register that holds cell charge rate
MAX1704X_VRESET_REG =  0x18  #///< Register that holds reset voltage setting
MAX1704X_CHIPID_REG =  0x19  #///< Register that holds semi-unique chip ID
MAX1704X_STATUS_REG =  0x1A  #///< Register that holds current alert/status
MAX1704X_CMD_REG =     0xFE  #///< Register that can be written for special commands

MAX1704X_ALERTFLAG_SOC_CHANGE =      0x20  #///< Alert flag for state-of-charge change
MAX1704X_ALERTFLAG_SOC_LOW    =      0x10  #///< Alert flag for state-of-charge low
MAX1704X_ALERTFLAG_VOLTAGE_RESET =   0x08  #///< Alert flag for voltage reset dip
MAX1704X_ALERTFLAG_VOLTAGE_LOW   =   0x04  #///< Alert flag for cell voltage low
MAX1704X_ALERTFLAG_VOLTAGE_HIGH =    0x02  #///< Alert flag for cell voltage high
MAX1704X_ALERTFLAG_RESET_INDICATOR = 0x01  #///< Alert flag for IC reset notification

#/*!
# *    @brief  Class that stores state and functions for interacting with
# *            the MAX17048 I2C battery monitor
# */
class MAX17048():
    def __init__(self, board):
        self.board = board
        self.addr = MAX17048_I2CADDR_DEFAULT
        self.board.set_pin_mode_i2c()
        self.data = {"ic_ver": "",
                     "chip_id": "",
                     "cell_voltage": "",
                     "cell_pct": "",
                     "chrg_rt ": "",
                     "rst_voltage": "",
                     "alt_voltage_hi": "",
                     "alt_voltage_lo": "",
                     "alert_status": "",
                     "act_thresh": "",
                     "hib_thresh": "",
                     "intf_rslt": "",
                     }


    #/*!
    #*    @brief  Sets up the hardware and initializes I2C
    #*    @param  wire
    #*            The Wire object to be used for I2C connections.
    #*    @return True if initialization was successful, otherwise false.
    #*/
    def begin(self):
        if ((self.getICversion() & 0xFFF0) != 0x10):  #// couldnt find the chip?
            return False

        if (not self.reset()):
            return False

        self.enableSleep(False)
        self.sleep(False)

        return True
    
    def i2c_write(self, reg_addr, reg_data):
        self.board.i2c_write(self.addr, [reg_addr, reg_data])
        self.i2c_read(reg_addr, 1)
        return 0

    def i2c_read(self, reg_addr, rlen):
        self.board.i2c_read(self.addr, reg_addr, rlen, self.read_callback)
        time.sleep(0.1)
        return 0

    def read_callback(self, data):
        self.data["intf_rslt"] = data

    #/*!
    #*    @brief  Get IC LSI version
    #*    @return 16-bit value read from MAX1704X_VERSION_REG register
    #*/
    def getICversion(self):
        self.i2c_read(MAX1704X_VERSION_REG, 2)
        ic_ver = (self.data["intf_rslt"][5] << 8) + self.data["intf_rslt"][6]
        self.data["ic_ver"] = ic_ver
        return ic_ver
        

    #/*!
    #*    @brief  Get semi-unique chip ID
    #*    @return 8-bit value read from MAX1704X_VERSION_REG register
    #*/
    def getChipID(self):
        self.i2c_read(MAX1704X_CHIPID_REG, 1)
        chip_id = self.data["intf_rslt"][5]
        self.data["chip_id"] = chip_id
        return chip_id
        

    #/*!
    #*    @brief  Soft reset the MAX1704x
    #*    @return True on reset success
    #*/
    def reset(self):
        self.i2c_write(MAX1704X_CMD_REG, 0x54)
        self.i2c_write(MAX1704X_CMD_REG + 1, 0x00)
        return self.clearAlertFlag(MAX1704X_ALERTFLAG_RESET_INDICATOR)
        

    #/*!
    #*    @brief  Function for clearing an alert flag once it has been handled.
    #*    @param flags A byte that can have any number of OR'ed alert flags:
    #*    MAX1704X_ALERTFLAG_SOC_CHANGE, MAX1704X_ALERTFLAG_SOC_LOW,
    #*    MAX1704X_ALERTFLAG_VOLTAGE_RESET, MAX1704X_ALERTFLAG_VOLTAGE_LOW
    #*    MAX1704X_ALERTFLAG_VOLTAGE_HIGH, or MAX1704X_ALERTFLAG_RESET_INDICATOR
    #*    @return True if the status register write succeeded
    #*/
    def clearAlertFlag(self, flags):
        self.i2c_write(MAX1704X_STATUS_REG, MAX1704X_STATUS_REG & ~flags)
    

    #/*!
    #*    @brief  Get battery voltage
    #*    @return Floating point value read in Volts
    #*/
    def cellVoltage(self):
        self.i2c_read(MAX1704X_VCELL_REG, 2)
        vcell = (self.data["intf_rslt"][5] << 8) + self.data["intf_rslt"][6]
        voltage = vcell * (78.125 / 1000000)
        self.data["cell_voltage"] = voltage
        return voltage


    #/*!
    #*    @brief  Get battery state in percent (0-100%)
    #*    @return Floating point value from 0 to 100.0
    #*/
    def cellPercent(self):
        self.i2c_read(MAX1704X_SOC_REG, 2)
        vperc = (self.data["intf_rslt"][5] << 8) + self.data["intf_rslt"][6]
        percent = (vperc / 256.0)
        self.data["cell_pct"] = percent
        return percent 
    

    #/*!
    #*    @brief  Charge or discharge rate of the battery in percent/hour
    #*    @return Floating point value from 0 to 100.0% per hour
    #*/
    def chargeRate(self):
        self.i2c_read(MAX1704X_CRATE_REG, 2)
        crate = (self.data["intf_rslt"][5] << 8) + self.data["intf_rslt"][6]
        percenthr = crate * 0.00208
        self.data["chrg_rt"] = percenthr
        return percenthr 
    

    #/*!
    #*    @brief Setter function for the voltage that the IC considers 'resetting'
    #*    @param reset_v Floating point voltage that, when we go below, should be
    #* considered a reset
    #*/

    def setResetVoltage(self, reset_v):
        self.i2c_read(MAX1704X_VRESET_REG, 1)
        vreset = self.data["intf_rslt"][5]
        reset_v = max(min(int(reset_v / 0.04), 127), 0) #// 40mV / LSB
        vreset = (0b11111110 & reset_v) + (0b1 & vreset)
        self.i2c_write(MAX1704X_VRESET_REG, vreset)
        self.getResetVoltage()

    #/*!
    #*    @brief Getter function for the voltage that the IC considers 'resetting'
    #*    @returns Floating point voltage that, when we go below, should be
    #* considered a reset
    #*/
    def getResetVoltage(self):
        self.i2c_read(MAX1704X_VRESET_REG, 1)
        vreset = self.data["intf_rslt"][5] & 0b11111110
        val = vreset * 0.04  #// 40mV / LSB
        self.data["rst_voltage"] = val
        return val


    #/*!
    #*    @brief Setter function for the voltage alert min/max settings
    #*    @param minv The minimum voltage: alert if we go below
    #*    @param maxv The maximum voltage: alert if we go above
    #*/
    def setAlertVoltages(self, minv, maxv):
        minv_int = min(255, max(0, int(minv / 0.02)))
        maxv_int = min(255, max(0, int(maxv / 0.02)))
        self.i2c_write(MAX1704X_VALERT_REG, minv_int)
        self.i2c_write(MAX1704X_VALERT_REG + 1, maxv_int)


    #/*!
    #*    @brief Getter function for the voltage alert min/max settings
    #*    @param minv The minimum voltage: alert if we go below
    #*    @param maxv The maximum voltage: alert if we go above
    #*/
    def getAlertVoltages(self):
        self.i2c_read(MAX1704X_VALERT_REG, 1)
        minv = self.data["intf_rslt"][5] * 0.02 #// 20mV / LSB
        self.i2c_read(MAX1704X_VALERT_REG + 1, 1)
        maxv = self.data["intf_rslt"][5] * 0.02 #// 20mV / LSB
        self.data["alt_voltage_hi"] = maxv
        self.data["alt_voltage_lo"] = minv
        return minv, maxv

    #/*!
    #*    @brief A check to determine if there is an unhandled alert
    #*    @returns True if there is an alert status flag
    #*/
    def isActiveAlert(self):
        self.i2c_read(MAX1704X_CONFIG_REG, 2)
        config_reg = (self.data["intf_rslt"][5] << 8) + self.data["intf_rslt"][6]
        alert_bit = (config_reg & 0b0000000000100000)
        self.data["alert_status"] = alert_bit
        return bool(alert_bit)
        

    #/*!
    #*    @brief Get all 7 alert flags from the status register in a uint8_t
    #*    @returns A byte that has all 7 alert flags. You can then check the flags
    #*    MAX1704X_ALERTFLAG_SOC_CHANGE, MAX1704X_ALERTFLAG_SOC_LOW,
    #*    MAX1704X_ALERTFLAG_VOLTAGE_RESET, MAX1704X_ALERTFLAG_VOLTAGE_LOW
    #*    MAX1704X_ALERTFLAG_VOLTAGE_HIGH, or MAX1704X_ALERTFLAG_RESET_INDICATOR
    #*/
    def getAlertStatus(self):
        self.i2c_read(MAX1704X_STATUS_REG, 2)
        self.data["alert_status"] = (self.data["intf_rslt"][5] << 8) +  self.data["intf_rslt"][6]
        return self.data["alert_status"] & 0x7F
        

    #/*!
    #*    @brief The voltage change that will trigger exiting hibernation mode.
    #*    If at any ADC sample abs(OCVCELL) is greater than ActThr, the IC exits
    #*    hibernate mode.
    #*    @returns The threshold, from 0-0.31874 V that will be used to determine
    #*    whether its time to exit hibernation.
    #*/
    def getActivityThreshold(self):
        self.i2c_read(MAX1704X_HIBRT_REG + 1, 1)
        activityThreshold = self.data["intf_rslt"][5] * 0.00125  #// 1.25mV per LSB
        self.data["act_thresh"] = activityThreshold
        return activityThreshold
    

    #/*!
    #*    @brief Set the voltage change that will trigger exiting hibernation mode.
    #*    If at any ADC sample abs(OCVCELL) is greater than ActThr, the IC exits
    #*    hibernate mode.
    #*    @param actthresh The threshold voltage, from 0-0.31874 V that will be
    #*    used to determine whether its time to exit hibernation.
    #*/
    def setActivityThreshold(self, actthresh):
        self.i2c_write(MAX1704X_HIBRT_REG + 1, min(255, max(0, int(actthresh / 0.00125))))  #// 1.25mV per LSB
        self.getActivityThreshold()
    

    #/*!
    #*    @brief The %/hour change that will trigger hibernation mode. If the
    #*    absolute value of CRATE is less than HibThr for longer than 6min,
    #*    the IC enters hibernate mode
    #*    @returns The threshold, from 0-53% that will be used to determine
    #*    whether its time to hibernate.
    #*/
    def getHibernationThreshold(self):
        self.i2c_read(MAX1704X_HIBRT_REG, 1)
        hibernationThreshold = self.data["intf_rslt"][5] * 0.208  #// 0.208% per hour
        self.data["hib_thresh"] = hibernationThreshold
        return hibernationThreshold


    #/*!
    #*    @brief Determine the %/hour change that will trigger hibernation mode
    #*    If the absolute value of CRATE is less than HibThr for longer than 6min,
    #*    the IC enters hibernate mode
    #*    @param hibthresh The threshold, from 0-53% that will be used to determine
    #*    whether its time to hibernate.
    #*/
    def setHibernationThreshold(self, hibthresh):
        self.i2c_write(MAX1704X_HIBRT_REG, min(255, max(0, int(hibthresh / 0.208))))  #// 0.208% per hour
        self.getHibernationThreshold()
        

    #/*!
    #*    @brief Query whether the chip is hibernating now
    #*    @returns True if hibernating
    #*/
    def isHibernating(self):
        self.i2c_read(MAX1704X_MODE_REG, 1)
        hib_bit = self.data["intf_rslt"][5] & 0b10000
        return bool(hib_bit)
        

    #/*!
    #*    @brief Enter hibernation mode.
    #*/
    def hibernate(self):
        self.i2c_write(MAX1704X_HIBRT_REG + 1, 0xFF)
        self.i2c_write(MAX1704X_HIBRT_REG, 0xFF)
        

    #/*!
    #*    @brief Wake up from hibernation mode.
    #*/
    def wake(self):
        self.i2c_write(MAX1704X_HIBRT_REG + 1, 0x0)
        self.i2c_write(MAX1704X_HIBRT_REG, 0x0)


    #/*!
    #*    @brief Enter ultra-low-power sleep mode (1uA draw)
    #*    @param s True to force-enter sleep mode, False to leave sleep
    #*/
    def sleep(self, s):
        self.i2c_read(MAX1704X_CONFIG_REG + 1, 1)
        if s:
            config_reg = (self.data["intf_rslt"][5] & 0b01111111) + 128
        else:
            config_reg = (self.data["intf_rslt"][5] & 0b01111111)
        self.i2c_write(MAX1704X_CONFIG_REG + 1, config_reg)

    #/*!
    #*    @brief Enable the ability to enter ultra-low-power sleep mode (1uA draw)
    #*    @param en True to enable sleep mode, False to only allow hibernation
    #*/
    def enableSleep(self, en):
        self.i2c_read(MAX1704X_MODE_REG, 1)
        if en:
            mode_reg = (self.data["intf_rslt"][5] & 0b01010000) + 32
        else:
            mode_reg = (self.data["intf_rslt"][5] & 0b01010000)
        self.i2c_write(MAX1704X_MODE_REG, mode_reg)
        

    #/*!
    #*    @brief  Quick starting allows an instant 'auto-calibration' of the
    #     battery. However, its a bad idea to do this right when the battery is first
    #     plugged in or if there's a lot of load on the battery so uncomment only if
    #     you're sure you want to 'reset' the chips charge calculator.
    #*/
    #def quickStart(self):
    #    self.i2c_read(MAX1704X_MODE_REG, 1)
    #    mode_reg = (self.data["intf_rslt"][5] & 0b00110000) + 64
    #    self.i2c_write(MAX1704X_MODE_REG, mode_reg)
