#/**
#* Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
#*
#* BSD-3-Clause
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*
#* 1. Redistributions of source code must retain the above copyright
#*    notice, this list of conditions and the following disclaimer.
#*
#* 2. Redistributions in binary form must reproduce the above copyright
#*    notice, this list of conditions and the following disclaimer in the
#*    documentation and/or other materials provided with the distribution.
#*
#* 3. Neither the name of the copyright holder nor the names of its
#*    contributors may be used to endorse or promote products derived from
#*    this software without specific prior written permission.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#* SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
#* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
#* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#* POSSIBILITY OF SUCH DAMAGE.
#*
#* @file       bme68x.c
#* @date       2021-05-24
#* @version    v4.4.6
#*
#/

from BME688.bme688_defs import *
import time

#/******************************************************************************************/
#/*                                 Global API definitions                                 */
#/******************************************************************************************/

#/* @brief This API reads the chip-id of the sensor which is the first step to
#* verify the sensor and also calibrates the sensor
#* As this API is the entry point, call this API before using other APIs.
#*/

class BME68x():
    def __init__(self):
        pass
    
    def begin(self, dev):
        rslt = None

        rslt = self.bme68x_soft_reset(dev)
        if (rslt == BME68X_OK):
            self.bme68x_get_regs(BME68X_REG_CHIP_ID, 1, dev)
            time.sleep(0.1)
            dev = dev["device"]()
            dev["chip_id"] = dev["intf_rslt"][5]
            if (rslt == BME68X_OK):
                if (dev["chip_id"] == BME68X_CHIP_ID):
                    #/* Read Variant ID */
                    rslt = self._read_variant_id(dev)
                    if (rslt == BME68X_OK):
                        #/* Get the Calibration data */
                        rslt = self._get_calib_data(dev)
                else:
                    rslt = BME68X_E_DEV_NOT_FOUND
        return rslt

    #/*
    # * @brief This API writes the given data to the register address of the sensor
    # */
    def bme68x_set_regs(self, reg_addr, reg_data, dlen, dev):
        rslt = None

        #/* Length of the temporary buffer is 2*(length of register)*/
        tmp_buff = []
        index = 0
        #print(reg_addr, reg_data, dlen)
        #/* Check for null pointer in the device structure*/
        rslt = self._null_ptr_check(dev)
        if ((rslt == BME68X_OK) and reg_addr and reg_data):
            if ((dlen > 0) and (dlen <= (BME68X_LEN_INTERLEAVE_BUFF / 2))):
                for index in range(dlen):
                    if (rslt == BME68X_OK):
                        dev["intf_rslt"] = dev["write"](reg_addr[index], reg_data[index], dev)
                        if (dev["intf_rslt"] != 0):
                            rslt = BME68X_E_COM_FAIL
                            break
            else:
                rslt = BME68X_E_INVALID_LENGTH
        else:
            rslt = BME68X_E_NULL_PTR
        return rslt

    #/*
    #* @brief This API reads the data from the given register address of sensor.
    #*/
    def bme68x_get_regs(self, reg_addr, rlen, dev):
        rslt = None
        #/* Check for null pointer in the device structure*/
        rslt = self._null_ptr_check(dev)
        if (rslt == BME68X_OK):
            dev["intf_rslt"] = dev["read"](reg_addr, rlen, dev)
            if (dev["intf_rslt"] != 0):
                rslt = BME68X_E_COM_FAIL
        else:
            rslt = BME68X_E_NULL_PTR
        return rslt
    
    #/*
    #* @brief This API soft-resets the sensor.
    #*/
    def bme68x_soft_reset(self, dev):
        rslt = None
        reg_addr = BME68X_REG_SOFT_RESET

        #/* 0xb6 is the soft reset command */
        soft_rst_cmd = BME68X_SOFT_RESET_CMD

        #/* Check for null pointer in the device structure*/
        rslt = self._null_ptr_check(dev)
        if (rslt == BME68X_OK):
            #/* Reset the device */
            if (rslt == BME68X_OK):
                rslt = self.bme68x_set_regs([reg_addr], [soft_rst_cmd], 1, dev)
                #/* Wait for 5ms */
                time.sleep(0.005)
            
        return rslt

    #/*
    #* @brief This API is used to set the oversampling, filter and odr configuration
    #*/
    def bme68x_set_conf(self, conf, dev):
        rslt = None
        odr20 = 0
        odr3 = 1
        current_op_mode = None

        #/* Register data starting from BME68X_REG_CTRL_GAS_1(0x71) up to BME68X_REG_CONFIG(0x75) */
        reg_array =  [0x71, 0x72, 0x73, 0x74, 0x75]
        data_array =  [] 

        rslt = self.bme68x_get_op_mode(dev)
        current_op_mode = dev["op_mode"]
        if (rslt == BME68X_OK):
            #/* Configure only in the sleep mode */
            rslt = self.bme68x_set_op_mode(BME68X_SLEEP_MODE, dev)
        
        if (conf == None):
            rslt = BME68X_E_NULL_PTR
        
        elif (rslt == BME68X_OK):
            #/* Read the whole configuration and write it back once later */
            rslt = self.bme68x_get_regs(reg_array[0], BME68X_LEN_CONFIG, dev)
            time.sleep(0.1)
            data_array = dev["intf_rslt"][5: BME68X_LEN_CONFIG + 5]
            dev["info_msg"] = BME68X_OK
            if (rslt == BME68X_OK):
                rslt = self._boundary_check(conf["filter"], BME68X_FILTER_SIZE_127, dev)
            if (rslt == BME68X_OK):
                rslt = self._boundary_check(conf["os_temp"], BME68X_OS_16X, dev)
            if (rslt == BME68X_OK):
                rslt = self._boundary_check(conf["os_pres"], BME68X_OS_16X, dev)
            if (rslt == BME68X_OK):
                rslt = self._boundary_check(conf["os_hum"], BME68X_OS_16X, dev)
            if (rslt == BME68X_OK):
                rslt = self._boundary_check(conf["odr"], BME68X_ODR_NONE, dev)
            if (rslt == BME68X_OK):
                data_array[4] = BME68X_SET_BITS(data_array[4], "BME68X_FILTER", conf["filter"])
                data_array[3] = BME68X_SET_BITS(data_array[3], "BME68X_OST", conf["os_temp"])
                data_array[3] = BME68X_SET_BITS(data_array[3], "BME68X_OSP", conf["os_pres"])
                data_array[1] = BME68X_SET_BITS_POS_0(data_array[1], "BME68X_OSH", conf["os_hum"])
                if (conf["odr"] != BME68X_ODR_NONE):
                    odr20 = conf["odr"]
                    odr3 = 0
                
                data_array[4] = BME68X_SET_BITS(data_array[4], "BME68X_ODR20", odr20)
                data_array[0] = BME68X_SET_BITS(data_array[0], "BME68X_ODR3", odr3)
            
        if (rslt == BME68X_OK):
            rslt = self.bme68x_set_regs(reg_array, data_array, BME68X_LEN_CONFIG, dev)
        if ((current_op_mode != BME68X_SLEEP_MODE) and (rslt == BME68X_OK)):
            rslt = self.bme68x_set_op_mode(current_op_mode, dev)
        
        return rslt

    #/*
    #* @brief This API is used to get the oversampling, filter and odr
    #*/
    def bme68x_get_conf(self, conf, dev):
        rslt = None

        #/* starting address of the register array for burst read*/
        reg_addr = BME68X_REG_CTRL_GAS_1
        data_array = []

        rslt = self.bme68x_get_regs(reg_addr, 5, dev)
        time.sleep(0.1)
        data_array = dev["intf_rslt"][5:10]
        if (not conf):
            rslt = BME68X_E_NULL_PTR
        elif (rslt == BME68X_OK):
            conf["os_hum"] = BME68X_GET_BITS_POS_0(data_array[1], "BME68X_OSH")
            conf["filter"] = BME68X_GET_BITS(data_array[4], "BME68X_FILTER")
            conf["os_temp"] = BME68X_GET_BITS(data_array[3], "BME68X_OST")
            conf["os_pres"] = BME68X_GET_BITS(data_array[3], "BME68X_OSP")
            if (BME68X_GET_BITS(data_array[0], "BME68X_ODR3")):
                conf["odr"] = BME68X_ODR_NONE
            else:
                conf["odr"] = BME68X_GET_BITS(data_array[4], "BME68X_ODR20")
        return rslt
    
    #/*
    #* @brief This API is used to set the operation mode of the sensor
    #*/
    def bme68x_set_op_mode(self, op_mode, dev):
        rslt = 0
        tmp_pow_mode = None
        pow_mode = -1
        reg_addr = BME68X_REG_CTRL_MEAS

        #/* Call until in sleep */
        while ((pow_mode != BME68X_SLEEP_MODE) and (rslt == BME68X_OK)):
            rslt = self.bme68x_get_regs(BME68X_REG_CTRL_MEAS, 1, dev)
            if (rslt == BME68X_OK):
                time.sleep(0.1)
                tmp_pow_mode = dev["intf_rslt"][5]
                #/* Put to sleep before changing mode */
                pow_mode = (tmp_pow_mode & BME68X_MODE_MSK)
                if (pow_mode != BME68X_SLEEP_MODE):
                    tmp_pow_mode &= ~BME68X_MODE_MSK #/* Set to sleep */
                    rslt = self.bme68x_set_regs([reg_addr], [tmp_pow_mode], 1, dev)
                    time.sleep(0.005)
                
        #/* Already in sleep */
        if ((op_mode != BME68X_SLEEP_MODE) and (rslt == BME68X_OK)):
            tmp_pow_mode = (tmp_pow_mode & ~BME68X_MODE_MSK) | (op_mode & BME68X_MODE_MSK)
            rslt = self.bme68x_set_regs([reg_addr], [tmp_pow_mode], 1, dev)
        return rslt
    
    #/*
    #* @brief This API is used to get the operation mode of the sensor.
    #*/
    def bme68x_get_op_mode(self, dev):
        rslt = None
        mode = None

        rslt = self.bme68x_get_regs(BME68X_REG_CTRL_MEAS, 1, dev)
        time.sleep(0.1)
        mode = dev["intf_rslt"][5]
        #/* Masking the other register bit info*/
        dev["op_mode"] = mode & BME68X_MODE_MSK
        
        return rslt
    
    #/*
    #* @brief This API is used to get the remaining duration that can be used for heating.
    #*/
    def bme68x_get_meas_dur(self, op_mode, conf, dev):
        rslt = None
        meas_dur = 0  #/* Calculate in us */
        meas_cycles = None
        os_to_meas_cycles =  [0, 1, 2, 4, 8, 16] 

        if (conf != None):
            #/* Boundary check for temperature oversampling */
            rslt = self._boundary_check(conf["os_temp"], BME68X_OS_16X, dev)

            if (rslt == BME68X_OK):
                #/* Boundary check for pressure oversampling */
                rslt = self._boundary_check(conf["os_pres"], BME68X_OS_16X, dev)
            if (rslt == BME68X_OK):
                #/* Boundary check for humidity oversampling */
                rslt = self._boundary_check(conf["os_hum"], BME68X_OS_16X, dev)
            if (rslt == BME68X_OK):
                meas_cycles = os_to_meas_cycles[conf["os_temp"]]
                meas_cycles += os_to_meas_cycles[conf["os_pres"]]
                meas_cycles += os_to_meas_cycles[conf["os_hum"]]

                #/* TPH measurement duration */
                meas_dur = meas_cycles * 1963
                meas_dur += 477 * 4 #/* TPH switching duration */
                meas_dur += 477 * 5 #/* Gas measurement duration */

                if (op_mode != BME68X_PARALLEL_MODE):
                    meas_dur += 1000 #/* Wake up duration of 1ms */

        return meas_dur
    
    #/*
    #* @brief This API reads the pressure, temperature and humidity and gas data
    #* from the sensor, compensates the data and store it in the bme68x_data
    #* structure instance passed by the user.
    #*/
    def bme68x_get_data(self, op_mode, data, n_data, dev):
        rslt = None
        i = 0
        j = 0
        new_fields = 0
        field_ptr =  [0, 0, 0] 
        field_data =   [0, 0, 0]  

        field_ptr[0] = field_data[0]
        field_ptr[1] = field_data[1]
        field_ptr[2] = field_data[2]

        rslt = self._null_ptr_check(dev)
        if ((rslt == BME68X_OK) and (data != None)):
            #/* Reading the sensor data in forced mode only */
            if (op_mode == BME68X_FORCED_MODE):
                rslt = self._read_field_data(0, data, dev)
                if (rslt == BME68X_OK):
                    if (data["status"] & BME68X_NEW_DATA_MSK):
                        new_fields = 1
                    else:
                        new_fields = 0
                        rslt = BME68X_W_NO_NEW_DATA
                    
            elif ((op_mode == BME68X_PARALLEL_MODE) and (op_mode == BME68X_SEQUENTIAL_MODE)):
                #/* Read the 3 fields and count the number of new data fields */
                rslt = self._read_all_field_data(field_ptr, dev)
                new_fields = 0
                i = 0
                while ((i < 3) and (rslt == BME68X_OK)):
                    if (field_ptr[i]["status"] & BME68X_NEW_DATA_MSK):
                        new_fields += 1
                    i += 1
                    
                #/* Sort the sensor data in parallel & sequential modes*/
                i = 0
                while ((i < 2) and (rslt == BME68X_OK)):
                    j = i + 1
                    while ( j < 3):
                        self._sort_sensor_data(i, j, field_ptr)
                        j += 1
                #/* Copy the sorted data */
                i = 0
                while (((i < 3) and (rslt == BME68X_OK))):
                    data[i] = field_ptr[i]
                    i += 1
                
                if (new_fields == 0):
                    rslt = BME68X_W_NO_NEW_DATA
                
            else:
                rslt = BME68X_W_DEFINE_OP_MODE
        
            if (n_data == None):
                rslt = BME68X_E_NULL_PTR
            else:
                n_data = new_fields
        else:
            rslt = BME68X_E_NULL_PTR

        return rslt
    
    #/*
    #* @brief This API is used to set the gas configuration of the sensor.
    #*/
    def bme68x_set_heatr_conf(self, op_mode, conf, dev):
        rslt = None
        nb_conv = 0
        hctrl = 0
        run_gas = 0
        ctrl_gas_data = [0, 0]
        ctrl_gas_addr =  [BME68X_REG_CTRL_GAS_0, BME68X_REG_CTRL_GAS_1] 

        if (conf != None):
            rslt = self.bme68x_set_op_mode(BME68X_SLEEP_MODE, dev)
            if (rslt == BME68X_OK):
                rslt = self._set_conf(conf, op_mode, nb_conv, dev)
            if (rslt == BME68X_OK):
                rslt = self.bme68x_get_regs(BME68X_REG_CTRL_GAS_0, 2, dev)
                if (rslt == BME68X_OK):
                    time.sleep(0.1)
                    ctrl_gas_data = dev["intf_rslt"][5:7]
                    if (conf["enable"] == BME68X_ENABLE):
                        hctrl = BME68X_ENABLE_HEATER
                        if (dev["variant_id"] == BME68X_VARIANT_GAS_HIGH):
                            run_gas = BME68X_ENABLE_GAS_MEAS_H
                        else:
                            run_gas = BME68X_ENABLE_GAS_MEAS_L
                    else:
                        hctrl = BME68X_DISABLE_HEATER
                        run_gas = BME68X_DISABLE_GAS_MEAS
                    
                    ctrl_gas_data[0] = BME68X_SET_BITS(ctrl_gas_data[0], "BME68X_HCTRL", hctrl)
                    ctrl_gas_data[1] = BME68X_SET_BITS_POS_0(ctrl_gas_data[1], "BME68X_NBCONV", nb_conv)
                    ctrl_gas_data[1] = BME68X_SET_BITS(ctrl_gas_data[1], "BME68X_RUN_GAS", run_gas)
                    rslt = self.bme68x_set_regs(ctrl_gas_addr, ctrl_gas_data, 2, dev)
        else:
            rslt = BME68X_E_NULL_PTR
        return rslt
    
    #/*
    #* @brief This API is used to get the gas configuration of the sensor.
    #*/
    def bme68x_get_heatr_conf(self, conf, dev):
        rslt = None
        data_array =  [] 

        #/* FIXME: Add conversion to deg C and ms and add the other parameters */
        rslt = self.bme68x_get_regs(BME68X_REG_RES_HEAT0, 10, dev)
        time.sleep(0.1)
        data_array = dev["intf_rslt"][5:11]
        if (rslt == BME68X_OK):
            if (conf and conf["heatr_dur_prof"] and conf["heatr_temp_prof"]):
                
                for i in range(10):
                    conf["heatr_temp_prof"][i] = data_array[i]
                
                rslt = self.bme68x_get_regs(BME68X_REG_GAS_WAIT0, 10, dev)
                time.sleep(0.1)
                data_array = dev["intf_rslt"][5:11]
                if (rslt == BME68X_OK):
                    for i in range(10):
                        conf["heatr_dur_prof"][i] = data_array[i]
            else:
                rslt = BME68X_E_NULL_PTR
        return rslt
    
    #/*
    #* @brief This API performs Self-test of low gas variant of BME68X
    #*/
    def bme68x_low_gas_selftest_check(self, dev):
        return "skip"
        rslt = None
        n_fields = None
        i = 0
        data =   []  
        t_dev = {}
        conf = None
        heatr_conf = None

        #/* Copy required parameters from reference bme68x_dev struct */
        t_dev["amb_temp"] = 25
        t_dev["read"] = dev["read"]
        t_dev["write"] = dev["write"]
        t_dev["intf"] = dev["intf"]
        t_dev["delay_us"] = dev["delay_us"]
        t_dev["intf_ptr"] = dev["intf_ptr"]
        rslt = self.__init__(t_dev)
        if (rslt == BME68X_OK):
            #/* Set the temperature, pressure and humidity & filter settings */
            conf.os_hum = BME68X_OS_1X
            conf.os_pres = BME68X_OS_16X
            conf.os_temp = BME68X_OS_2X

            #/* Set the remaining gas sensor settings and link the heating profile */
            heatr_conf.enable = BME68X_ENABLE
            heatr_conf.heatr_dur = BME68X_HEATR_DUR1
            heatr_conf.heatr_temp = BME68X_HIGH_TEMP
            rslt = self.bme68x_set_heatr_conf(BME68X_FORCED_MODE, heatr_conf, t_dev)
            if (rslt == BME68X_OK):
                rslt = self.bme68x_set_conf(conf, t_dev)
                if (rslt == BME68X_OK):
                    rslt = self.bme68x_set_op_mode(BME68X_FORCED_MODE, t_dev) #/* Trigger a measurement */
                    if (rslt == BME68X_OK):
                        #/* Wait for the measurement to complete */
                        t_dev.delay_us(BME68X_HEATR_DUR1_DELAY, t_dev.intf_ptr)
                        rslt = self.bme68x_get_data(BME68X_FORCED_MODE, data[0], n_fields, t_dev)
                        if (rslt == BME68X_OK):
                            if ((data[0].idac != 0x00) and (data[0].idac != 0xFF) and (data[0].status & BME68X_GASM_VALID_MSK)):
                                rslt = BME68X_OK
                            else:
                                rslt = BME68X_E_SELF_TEST

            heatr_conf.heatr_dur = BME68X_HEATR_DUR2
            while ((rslt == BME68X_OK) and (i < BME68X_N_MEAS)):
                if (i % 2 == 0):
                    heatr_conf.heatr_temp = BME68X_HIGH_TEMP #/* Higher temperature */
                else:
                    heatr_conf.heatr_temp = BME68X_LOW_TEMP #/* Lower temperature */
                
                rslt = self.bme68x_set_heatr_conf(BME68X_FORCED_MODE, heatr_conf, t_dev)
                if (rslt == BME68X_OK):
                    rslt = self.bme68x_set_conf(conf, t_dev)
                    if (rslt == BME68X_OK):
                        rslt = self.bme68x_set_op_mode(BME68X_FORCED_MODE, t_dev) #/* Trigger a measurement */
                        if (rslt == BME68X_OK):
                            #/* Wait for the measurement to complete */
                            t_dev.delay_us(BME68X_HEATR_DUR2_DELAY, t_dev.intf_ptr)
                            rslt = self.bme68x_get_data(BME68X_FORCED_MODE, data[i], n_fields, t_dev)
                i += 1
            
            if (rslt == BME68X_OK):
                rslt = self._analyze_sensor_data(data, BME68X_N_MEAS)

        return rslt
    

    #/*****************************INTERNAL APIs***********************************************/

    #/* @brief This internal API is used to calculate the temperature value. */
    def _calc_temperature(self, temp_adc, dev):
        var1 = 0.0
        var2 = 0.0
        calc_temp = 0.0

        #/* calculate var1 data */
        var1 = (((temp_adc / 16384.0) - (dev["calib"]["par_t1"] / 1024.0)) * (dev["calib"]["par_t2"]))

        #/* calculate var2 data */
        var2 = ((((temp_adc / 131072.0) - (dev["calib"]["par_t1"] / 8192.0)) * ((temp_adc / 131072.0) - (dev["calib"]["par_t1"] / 8192.0))) * (dev["calib"]["par_t3"] * 16.0))

        #/* t_fine value*/
        dev["calib"]["t_fine"] = (var1 + var2)

        #/* compensated temperature data*/
        calc_temp = ((dev["calib"]["t_fine"]) / 5120.0)

        return calc_temp
    

    #/* @brief This internal API is used to calculate the pressure value. */
    def _calc_pressure(self, pres_adc, dev):
        var1 = 0.0
        var2 = 0.0
        var3 = 0.0
        calc_pres = 0.0

        var1 = ((dev["calib"]["t_fine"] / 2.0) - 64000.0)
        var2 = var1 * var1 * ((dev["calib"]["par_p6"]) / (131072.0))
        var2 = var2 + (var1 * (dev["calib"]["par_p5"]) * 2.0)
        var2 = (var2 / 4.0) + ((dev["calib"]["par_p4"]) * 65536.0)
        var1 = ((((dev["calib"]["par_p3"] * var1 * var1) / 16384.0) + (dev["calib"]["par_p2"] * var1)) / 524288.0)
        var1 = ((1.0 + (var1 / 32768.0)) * (dev["calib"]["par_p1"]))
        calc_pres = (1048576.0 - (pres_adc))

        #/* Avoid exception caused by division by zero */
        if (var1 != 0):
            calc_pres = (((calc_pres - (var2 / 4096.0)) * 6250.0) / var1)
            var1 = ((dev["calib"]["par_p9"]) * calc_pres * calc_pres) / 2147483648.0
            var2 = calc_pres * ((dev["calib"]["par_p8"]) / 32768.0)
            var3 = ((calc_pres / 256.0) * (calc_pres / 256.0) * (calc_pres / 256.0) * (dev["calib"]["par_p10"] / 131072.0))
            calc_pres = (calc_pres + (var1 + var2 + var3 + (dev["calib"]["par_p7"] * 128.0)) / 16.0)
        else:
            calc_pres = 0
        
        return calc_pres
    

    #/* This internal API is used to calculate the humidity in integer */
    def _calc_humidity(self, hum_adc, dev):
        calc_hum = 0.0
        var1 = 0.0
        var2 = 0.0
        var3 = 0.0
        var4 = 0.0
        temp_comp = 0.0

        #/* compensated temperature data*/
        temp_comp = ((dev["calib"]["t_fine"]) / 5120.0)
        var1 = hum_adc - ((dev["calib"]["par_h1"] * 16.0) + ((dev["calib"]["par_h3"] / 2.0) * temp_comp))
        var2 = var1 * (((dev["calib"]["par_h2"] / 262144.0) * \
                        (1.0 + ((dev["calib"]["par_h4"] / 16384.0) * temp_comp) + \
                        ((dev["calib"]["par_h5"] / 1048576.0) * temp_comp * temp_comp))))
        var3 = dev["calib"]["par_h6"] / 16384.0
        var4 = dev["calib"]["par_h7"] / 2097152.0
        calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2)
        if (calc_hum > 100.0):
            calc_hum = 100.0
        elif (calc_hum < 0.0):
            calc_hum = 0.0
        
        return calc_hum
    

    #/* This internal API is used to calculate the gas resistance low value in float */
    def _calc_gas_resistance_low(self, gas_res_adc, gas_range, dev):
        calc_gas_res = 0.0
        var1 = 0.0
        var2 = 0.0
        var3 = 0.0
        gas_res_f = gas_res_adc
        gas_range_f = (1 << gas_range) # /*lint !e790 / Suspicious truncation, integral to float */
        lookup_k1_range = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0]
        lookup_k2_range = [0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        var1 = (1340.0 + (5.0 * dev["calib"]["range_sw_err"]))
        var2 = (var1) * (1.0 + lookup_k1_range[gas_range] / 100.0)
        var3 = 1.0 + (lookup_k2_range[gas_range] / 100.0)
        calc_gas_res = 1.0 / (var3 * (0.000000125) * gas_range_f * (((gas_res_f - 512.0) / var2) + 1.0))

        return calc_gas_res
    

    #/* This internal API is used to calculate the gas resistance value in float */
    def _calc_gas_resistance_high(self, gas_res_adc, gas_range):
        calc_gas_res  = 0.0
        var1 = 262144 >> gas_range
        var2 = gas_res_adc - 512

        var2 *= 3
        var2 = 4096 + var2

        calc_gas_res = 1000000.0 * var1 / var2

        return calc_gas_res
    

    #/* This internal API is used to calculate the heater resistance value */
    def _calc_res_heat(self, temp, dev):
        var1  = 0.0
        var2 = 0.0
        var3 = 0.0
        var4 = 0.0
        var5 = 0.0
        res_heat = 0

        if (temp > 400):  #/* Cap temperature */
            temp = 400
        
        var1 = ((dev["calib"]["par_gh1"] / (16.0)) + 49.0)
        var2 = (((dev["calib"]["par_gh2"] / (32768.0)) * (0.0005)) + 0.00235)
        var3 = (dev["calib"]["par_gh3"] / (1024.0))
        var4 = (var1 * (1.0 + (var2 * temp)))
        var5 = (var4 + (var3 * dev["amb_temp"]))
        res_heat =(3.4 * ((var5 * (4 / (4 + dev["calib"]["res_heat_range"])) * (1 / (1 + (dev["calib"]["res_heat_val"] * 0.002)))) - 25))

        return res_heat
    

    #/* This internal API is used to calculate the gas wait */
    def _calc_gas_wait(self, dur):
        factor = 0
        durval = 0

        if (dur >= 0xfc0):
            durval = 0xff  #/* Max duration*/
        else:
            while (dur > 0x3F):
                dur = dur / 4
                factor += 1
            
            durval = (dur + (factor * 64))
        
        return durval
    

    #/* This internal API is used to read a single data of the sensor */
    def _read_field_data(self, index, data, dev):
        rslt = BME68X_OK
        buff =  []
        gas_range_l = 0
        gas_range_h = 0
        adc_temp = 0
        adc_pres = 0
        adc_hum = 0
        adc_gas_res_low = 0
        adc_gas_res_high = 0
        tries = 5

        while ((tries) and (rslt == BME68X_OK)):
        
            rslt = self.bme68x_get_regs(((BME68X_REG_FIELD0 + (index * BME68X_LEN_FIELD_OFFSET))), BME68X_LEN_FIELD, dev)
            time.sleep(0.1)
            buff = dev["intf_rslt"][5:18 + 5]
            if (not data):
                rslt = BME68X_E_NULL_PTR
                break
            
            data["status"] = buff[0] & BME68X_NEW_DATA_MSK
            data["gas_index"] = buff[0] & BME68X_GAS_INDEX_MSK
            data["meas_index"] = buff[1]

            #/* read the raw data from the sensor */
            adc_pres = ((buff[2] * 4096) | (buff[3] * 16) | int(buff[4] / 16))
            adc_temp = ((buff[5] * 4096) | (buff[6] * 16) | int(buff[7] / 16))
            adc_hum = ((buff[8] * 256) | buff[9])
            adc_gas_res_low = (buff[13] * 4 | int((buff[14]) / 64))
            adc_gas_res_high = (buff[15] * 4 | int((buff[16]) / 64))
            gas_range_l = buff[14] & BME68X_GAS_RANGE_MSK
            gas_range_h = buff[16] & BME68X_GAS_RANGE_MSK
            if (dev["variant_id"] == BME68X_VARIANT_GAS_HIGH):
                data["status"] |= buff[16] & BME68X_GASM_VALID_MSK
                data["status"] |= buff[16] & BME68X_HEAT_STAB_MSK
            else:
                data["status"] |= buff[14] & BME68X_GASM_VALID_MSK
                data["status"] |= buff[14] & BME68X_HEAT_STAB_MSK
            
            if ((data["status"] & BME68X_NEW_DATA_MSK) and (rslt == BME68X_OK)):
                rslt = self.bme68x_get_regs(BME68X_REG_RES_HEAT0 + data["gas_index"], 1, dev)
                if (rslt == BME68X_OK):
                    time.sleep(0.1)
                    data["res_heat"] = dev["intf_rslt"][5]
                    rslt = self.bme68x_get_regs(BME68X_REG_IDAC_HEAT0 + data["gas_index"], 1, dev)
                if (rslt == BME68X_OK):
                    time.sleep(0.1)
                    data["idac"] = dev["intf_rslt"][5]
                    rslt = self.bme68x_get_regs(BME68X_REG_GAS_WAIT0 + data["gas_index"], 1, dev)
                if (rslt == BME68X_OK):
                    time.sleep(0.1)
                    data["gas_wait"] = dev["intf_rslt"][5]
                    data["temperature"] = self._calc_temperature(adc_temp, dev)
                    data["pressure"] = self._calc_pressure(adc_pres, dev)
                    data["humidity"] = self._calc_humidity(adc_hum, dev)
                    if (dev["variant_id"] == BME68X_VARIANT_GAS_HIGH):
                        data["gas_resistance"] = self._calc_gas_resistance_high(adc_gas_res_high, gas_range_h)
                    else:
                        data["gas_resistance"] = self._calc_gas_resistance_low(adc_gas_res_low, gas_range_l, dev)
                    break
                
            if (rslt == BME68X_OK):
                dev["delay_us"](BME68X_PERIOD_POLL, dev["intf_ptr"])

            tries -= 1      
        return rslt
    

    #/* This internal API is used to read all data fields of the sensor */
    def _read_all_field_data(self, data, dev):
        rslt = BME68X_OK
        buff =  []
        gas_range_l = 0
        gas_range_h = 0
        adc_temp = 0
        adc_pres = 0
        adc_hum = 0
        adc_gas_res_low = 0
        adc_gas_res_high = 0
        off = 0
        set_val =  []  #/* idac, res_heat, gas_wait */
        i  = 0

        if (not data[0] and not data[1] and not data[2]):
            rslt = BME68X_E_NULL_PTR
        
        if (rslt == BME68X_OK):
            rslt = self.bme68x_get_regs(BME68X_REG_FIELD0, buff, BME68X_LEN_FIELD * 3, dev)
        
        if (rslt == BME68X_OK):
            rslt = self.bme68x_get_regs(BME68X_REG_IDAC_HEAT0, set_val, 30, dev)
        
        while (((i < 3) and (rslt == BME68X_OK))):
            off = (i * BME68X_LEN_FIELD)
            data[i].status = buff[off] & BME68X_NEW_DATA_MSK
            data[i].gas_index = buff[off] & BME68X_GAS_INDEX_MSK
            data[i].meas_index = buff[off + 1]

            #/* read the raw data from the sensor */
            adc_pres = (( buff[off + 2] * 4096) | (buff[off + 3] * 16) | (buff[off + 4] / 16))
            adc_temp = ((buff[off + 5] * 4096) | (buff[off + 6] * 16) | (buff[off + 7] / 16))
            adc_hum = ((buff[off + 8] * 256) | buff[off + 9])
            adc_gas_res_low = (buff[off + 13] * 4 | ((buff[off + 14]) / 64))
            adc_gas_res_high = (buff[off + 15] * 4 | ((buff[off + 16]) / 64))
            gas_range_l = buff[off + 14] & BME68X_GAS_RANGE_MSK
            gas_range_h = buff[off + 16] & BME68X_GAS_RANGE_MSK
            if (dev.variant_id == BME68X_VARIANT_GAS_HIGH):
                data[i].status |= buff[off + 16] & BME68X_GASM_VALID_MSK
                data[i].status |= buff[off + 16] & BME68X_HEAT_STAB_MSK
            else:
                data[i].status |= buff[off + 14] & BME68X_GASM_VALID_MSK
                data[i].status |= buff[off + 14] & BME68X_HEAT_STAB_MSK
            
            data[i].idac = set_val[data[i].gas_index]
            data[i].res_heat = set_val[10 + data[i].gas_index]
            data[i].gas_wait = set_val[20 + data[i].gas_index]
            data[i].temperature = self._calc_temperature(adc_temp, dev)
            data[i].pressure = self._calc_pressure(adc_pres, dev)
            data[i].humidity = self._calc_humidity(adc_hum, dev)
            if (dev.variant_id == BME68X_VARIANT_GAS_HIGH):
                data[i].gas_resistance = self._calc_gas_resistance_high(adc_gas_res_high, gas_range_h)
            else:
                data[i].gas_resistance = self._calc_gas_resistance_low(adc_gas_res_low, gas_range_l, dev)
            
            i += 1

        return rslt
    

    #/* This internal API is used to switch between SPI memory pages */
    def _set_mem_page(self, reg_addr, dev):
        rslt = None
        reg = 0
        mem_page = 0

        #/* Check for null pointers in the device structure*/
        rslt = self._null_ptr_check(dev)
        if (rslt == BME68X_OK):
            if (reg_addr > 0x7f):
                mem_page = BME68X_MEM_PAGE1
            else:
                mem_page = BME68X_MEM_PAGE0

            if (mem_page != dev.mem_page):
                dev.mem_page = mem_page
                dev.intf_rslt = dev.read(BME68X_REG_MEM_PAGE | BME68X_SPI_RD_MSK, reg, 1, dev.intf_ptr)
                if (dev.intf_rslt != 0):
                    rslt = BME68X_E_COM_FAIL
                
                if (rslt == BME68X_OK):
                    reg = reg & (~BME68X_MEM_PAGE_MSK)
                    reg = reg | (dev.mem_page & BME68X_MEM_PAGE_MSK)
                    dev.intf_rslt = dev.write(BME68X_REG_MEM_PAGE & BME68X_SPI_WR_MSK, reg, 1, dev.intf_ptr)
                    if (dev.intf_rslt != 0):
                        rslt = BME68X_E_COM_FAIL
        return rslt
    

    #/* This internal API is used to get the current SPI memory page */
    def _get_mem_page(self, dev):
        rslt = None
        reg = 0

        #/* Check for null pointer in the device structure*/
        rslt = self._null_ptr_check(dev)
        if (rslt == BME68X_OK):
            dev.intf_rslt = dev.read(BME68X_REG_MEM_PAGE | BME68X_SPI_RD_MSK, reg, 1, dev.intf_ptr)
            if (dev.intf_rslt != 0):
                rslt = BME68X_E_COM_FAIL
            else:
                dev.mem_page = reg & BME68X_MEM_PAGE_MSK
        return rslt
    

    #/* This internal API is used to limit the max value of a parameter */
    def _boundary_check(self, value, max, dev):
        rslt = None
        rslt = self._null_ptr_check(dev)
        if ((value != None) and (rslt == BME68X_OK)):
            #/* Check if value is above maximum value */
            if (value > max):
                #/* Auto correct the invalid value to maximum value */
                value = max
                dev["info_msg"] |= BME68X_I_PARAM_CORR
        else:
            rslt = BME68X_E_NULL_PTR
        return rslt
    

    #/* This internal API is used to check the bme68x_dev for null pointers */
    def _null_ptr_check(self, dev):
        rslt = BME68X_OK
        if ((dev == None) or (dev["read"] == None) or (dev["write"] == None) or (dev["delay_us"] == None)):
            #/* Device structure pointer is not valid */
            rslt = BME68X_E_NULL_PTR
        return rslt
    

    #/* This internal API is used to set heater configurations */
    def _set_conf(self, conf, op_mode, nb_conv, dev):
        rslt = BME68X_OK
        i = 0
        shared_dur = 0
        write_len = 0
        heater_dur_shared_addr = BME68X_REG_SHD_HEATR_DUR
        rh_reg_addr =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
        rh_reg_data =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
        gw_reg_addr =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
        gw_reg_data =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 

        
        if op_mode == BME68X_FORCED_MODE:
            rh_reg_addr[0] = BME68X_REG_RES_HEAT0
            rh_reg_data[0] = self._calc_res_heat(conf["heatr_temp"], dev)
            gw_reg_addr[0] = BME68X_REG_GAS_WAIT0
            gw_reg_data[0] = self._calc_gas_wait(conf["heatr_dur"])
            nb_conv = 0
            write_len = 1
        elif op_mode == BME68X_SEQUENTIAL_MODE:
            if (( not conf.heatr_dur_prof) or (not conf["heatr_temp_prof"])):
                rslt = BME68X_E_NULL_PTR
                return rslt
            
            while (i < conf["profile_len"]):
                rh_reg_addr[i] = BME68X_REG_RES_HEAT0 + i
                rh_reg_data[i] = self._calc_res_heat(conf["heatr_temp_prof"][i], dev)
                gw_reg_addr[i] = BME68X_REG_GAS_WAIT0 + i
                gw_reg_data[i] = self._calc_gas_wait(conf["heatr_dur_prof"][i])
                i += 1

            nb_conv = conf["profile_len"]
            write_len = conf["profile_len"]
        elif op_mode == BME68X_PARALLEL_MODE:
            if ((not conf["heatr_dur_prof"]) or (not conf["heatr_temp_prof"])):
                rslt = BME68X_E_NULL_PTR
                return rslt
            
            if (conf["shared_heatr_dur"] == 0):
                rslt = BME68X_W_DEFINE_SHD_HEATR_DUR
            
            while (i < conf["profile_len"]):
                rh_reg_addr[i] = BME68X_REG_RES_HEAT0 + i
                rh_reg_data[i] = self._calc_res_heat(conf["heatr_temp_prof"][i], dev)
                gw_reg_addr[i] = BME68X_REG_GAS_WAIT0 + i
                gw_reg_data[i] = conf["heatr_dur_prof"][i]
                i += 1

            nb_conv = conf["profile_len"]
            write_len = conf["profile_len"]
            shared_dur = self._calc_heatr_dur_shared(conf["shared_heatr_dur"])
            if (rslt == BME68X_OK):
                rslt = self.bme68x_set_regs(heater_dur_shared_addr, shared_dur, 1, dev)
        else:
            rslt = BME68X_W_DEFINE_OP_MODE
        

        if (rslt == BME68X_OK):
            rslt = self.bme68x_set_regs(rh_reg_addr, rh_reg_data, write_len, dev)
        
        if (rslt == BME68X_OK):
            rslt = self.bme68x_set_regs(gw_reg_addr, gw_reg_data, write_len, dev)
        
        return rslt
    

    #/* This internal API is used to calculate the register value for shared heater duration */
    def _calc_heatr_dur_shared(self, dur):
        factor = 0
        heatdurval = 0

        if (dur >= 0x783):
            heatdurval = 0xff  #/* Max duration */
        else:
            #/* Step size of 0.477ms */
            dur = ((dur * 1000) / 477)
            while (dur > 0x3F):
                dur = dur >> 2
                factor += 1
            heatdurval = (dur + (factor * 64))
    
        return heatdurval
    

    #/* This internal API is used sort the sensor data */
    def _sort_sensor_data(self, low_index, high_index, field):
        meas_index1 = 0
        meas_index2 = 0

        meas_index1 = field[low_index]["meas_index"]
        meas_index2 = field[high_index]["meas_index"]
        if ((field[low_index]["status"] & BME68X_NEW_DATA_MSK) and (field[high_index]["status"] & BME68X_NEW_DATA_MSK)):
            diff = meas_index2 - meas_index1
            if (((diff > -3) and (diff < 0)) or (diff > 2)):
                self._swap_fields(low_index, high_index, field)
        elif (field[high_index]["status"] & BME68X_NEW_DATA_MSK):
            self._swap_fields(low_index, high_index, field)
        

        #/* Sorting field data
        #*
        #* The 3 fields are filled in a fixed order with data in an incrementing
        #* 8-bit sub-measurement index which looks like
        #* Field index | Sub-meas index
        #*      0      |        0
        #*      1      |        1
        #*      2      |        2
        #*      0      |        3
        #*      1      |        4
        #*      2      |        5
        #*      ...
        #*      0      |        252
        #*      1      |        253
        #*      2      |        254
        #*      0      |        255
        #*      1      |        0
        #*      2      |        1
        #*
        #* The fields are sorted in a way so as to always deal with only a snapshot
        #* of comparing 2 fields at a time. The order being
        #* field0 & field1
        #* field0 & field2
        #* field1 & field2
        #* Here the oldest data should be in field0 while the newest is in field2.
        #* In the following documentation, field0's position would referred to as
        #* the lowest and field2 as the highest.
        #*
        #* In order to sort we have to consider the following cases,
        #*
        #* Case A: No fields have new data
        #*     Then do not sort, as this data has already been read.
        #*
        #* Case B: Higher field has new data
        #*     Then the new field get's the lowest position.
        #*
        #* Case C: Both fields have new data
        #*     We have to put the oldest sample in the lowest position. Since the
        #*     sub-meas index contains in essence the age of the sample, we calculate
        #*     the difference between the higher field and the lower field.
        #*     Here we have 3 sub-cases,
        #*     Case 1: Regular read without overwrite
        #*         Field index | Sub-meas index
        #*              0      |        3
        #*              1      |        4
        #*
        #*         Field index | Sub-meas index
        #*              0      |        3
        #*              2      |        5
        #*
        #*         The difference is always <= 2. There is no need to swap as the
        #*         oldest sample is already in the lowest position.
        #*
        #*     Case 2: Regular read with an overflow and without an overwrite
        #*         Field index | Sub-meas index
        #*              0      |        255
        #*              1      |        0
        #*
        #*         Field index | Sub-meas index
        #*              0      |        254
        #*              2      |        0
        #*
        #*         The difference is always <= -3. There is no need to swap as the
        #*         oldest sample is already in the lowest position.
        #*
        #*     Case 3: Regular read with overwrite
        #*         Field index | Sub-meas index
        #*              0      |        6
        #*              1      |        4
        #*
        #*         Field index | Sub-meas index
        #*              0      |        6
        #*              2      |        5
        #*
        #*         The difference is always > -3. There is a need to swap as the
        #*         oldest sample is not in the lowest position.
        #*
        #*     Case 4: Regular read with overwrite and overflow
        #*         Field index | Sub-meas index
        #*              0      |        0
        #*              1      |        254
        #*
        #*         Field index | Sub-meas index
        #*              0      |        0
        #*              2      |        255
        #*
        #*         The difference is always > 2. There is a need to swap as the
        #*         oldest sample is not in the lowest position.
        #*
        #* To summarize, we have to swap when
        #*     - The higher field has new data and the lower field does not.
        #*     - If both fields have new data, then the difference of sub-meas index
        #*         between the higher field and the lower field creates the
        #*         following condition for swapping.
        #*         - (diff > -3) && (diff < 0), combination of cases 1, 2, and 3.
        #*         - diff > 2, case 4.
        #*
        #*     Here the limits of -3 and 2 derive from the fact that there are 3 fields.
        #*     These values decrease or increase respectively if the number of fields increases.
        #*/
    

    #/* This internal API is used sort the sensor data */
    def _swap_fields(self, index1, index2, field):
        temp = field[index1]
        field[index1] = field[index2]
        field[index2] = temp
        return field
    

    #/* This Function is to analyze the sensor data */
    def _analyze_sensor_data(self, data, n_meas):
        rslt = BME68X_OK
        self_test_failed = 0
        i = 0
        cent_res = 0

        if ((data[0]["temperature"] < BME68X_MIN_TEMPERATURE) or (data[0]["temperature"] > BME68X_MAX_TEMPERATURE)):
            self_test_failed += 1
        
        if ((data[0]["pressure"] < BME68X_MIN_PRESSURE) or (data[0]["pressure"] > BME68X_MAX_PRESSURE)):
            self_test_failed += 1
        
        if ((data[0]["humidity"] < BME68X_MIN_HUMIDITY) or (data[0]["humidity"] > BME68X_MAX_HUMIDITY)):
            self_test_failed += 1
        
        while (i < n_meas):  #/* Every gas measurement should be valid */
            if (not (data[i]["status"] & BME68X_GASM_VALID_MSK)):
                self_test_failed += 1
            i += 1
        

        if (n_meas >= 6):        
            cent_res = ((5 * (data[3]["gas_resistance"] + data[5]["gas_resistance"])) / (2 * data[4]["gas_resistance"]))
        
        if (cent_res < 6):
            self_test_failed = 1
        
        if (self_test_failed):
            rslt = BME68X_E_SELF_TEST

        return rslt
    

    #/* This internal API is used to read the calibration coefficients */
    def _get_calib_data(self, dev):
        rslt = None
        coeff_array = []

        rslt = self.bme68x_get_regs(BME68X_REG_COEFF1, BME68X_LEN_COEFF1, dev)
        if (rslt == BME68X_OK):
            time.sleep(0.1)
            coeff_array = coeff_array + dev["intf_rslt"][5: BME68X_LEN_COEFF1 + 5]
            rslt = self.bme68x_get_regs(BME68X_REG_COEFF2, BME68X_LEN_COEFF2, dev)
        if (rslt == BME68X_OK):
            time.sleep(0.1)
            coeff_array = coeff_array + dev["intf_rslt"][5: BME68X_LEN_COEFF2 + 5]
            rslt = self.bme68x_get_regs(BME68X_REG_COEFF3, BME68X_LEN_COEFF3, dev)
        if (rslt == BME68X_OK):
            time.sleep(0.1)
            coeff_array = coeff_array + dev["intf_rslt"][5: BME68X_LEN_COEFF3 + 5]
            #/* Temperature related coefficients */
            #print(coeff_array)
            dev["calib"]["par_t1"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T1_MSB], coeff_array[BME68X_IDX_T1_LSB], False)
            dev["calib"]["par_t2"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T2_MSB], coeff_array[BME68X_IDX_T2_LSB])
            dev["calib"]["par_t3"] = twos_comp(coeff_array[BME68X_IDX_T3], 8)

            #/* Pressure related coefficients */
            dev["calib"]["par_p1"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P1_MSB], coeff_array[BME68X_IDX_P1_LSB], False)
            dev["calib"]["par_p2"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P2_MSB], coeff_array[BME68X_IDX_P2_LSB])
            dev["calib"]["par_p3"] = twos_comp(coeff_array[BME68X_IDX_P3], 8)
            dev["calib"]["par_p4"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P4_MSB], coeff_array[BME68X_IDX_P4_LSB])
            dev["calib"]["par_p5"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P5_MSB], coeff_array[BME68X_IDX_P5_LSB])
            dev["calib"]["par_p6"] = twos_comp(coeff_array[BME68X_IDX_P6], 8)
            dev["calib"]["par_p7"] = twos_comp(coeff_array[BME68X_IDX_P7], 8)
            dev["calib"]["par_p8"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P8_MSB], coeff_array[BME68X_IDX_P8_LSB])
            dev["calib"]["par_p9"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P9_MSB], coeff_array[BME68X_IDX_P9_LSB])
            dev["calib"]["par_p10"] = coeff_array[BME68X_IDX_P10]

            #/* Humidity related coefficients */
            dev["calib"]["par_h1"] = ((coeff_array[BME68X_IDX_H1_MSB] << 4) | (coeff_array[BME68X_IDX_H1_LSB] & BME68X_BIT_H1_DATA_MSK))
            dev["calib"]["par_h2"] = ((coeff_array[BME68X_IDX_H2_MSB] << 4) | ((coeff_array[BME68X_IDX_H2_LSB]) >> 4))
            dev["calib"]["par_h3"] = twos_comp(coeff_array[BME68X_IDX_H3], 8)
            dev["calib"]["par_h4"] = twos_comp(coeff_array[BME68X_IDX_H4], 8)
            dev["calib"]["par_h5"] = twos_comp(coeff_array[BME68X_IDX_H5], 8)
            dev["calib"]["par_h6"] = coeff_array[BME68X_IDX_H6]
            dev["calib"]["par_h7"] = twos_comp(coeff_array[BME68X_IDX_H7], 8)

            #/* Gas heater related coefficients */
            dev["calib"]["par_gh1"] = twos_comp(coeff_array[BME68X_IDX_GH1], 8)
            dev["calib"]["par_gh2"] = BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_GH2_MSB], coeff_array[BME68X_IDX_GH2_LSB])
            dev["calib"]["par_gh3"] = twos_comp(coeff_array[BME68X_IDX_GH3], 8)
            #/* Other coefficients */
            dev["calib"]["res_heat_range"] = (coeff_array[BME68X_IDX_RES_HEAT_RANGE] & BME68X_RHRANGE_MSK) / 16
            dev["calib"]["res_heat_val"] = twos_comp(coeff_array[BME68X_IDX_RES_HEAT_VAL], 8)
            dev["calib"]["range_sw_err"] = (twos_comp(coeff_array[BME68X_IDX_RANGE_SW_ERR] & BME68X_RSERROR_MSK, 8)) / 16
            dev["set_device"](dev)
        return rslt

    #/* This internal API is used to read variant ID information from the register */
    def _read_variant_id(self, dev):
        rslt = None

        #/* Read variant ID information register */
        rslt = self.bme68x_get_regs(BME68X_REG_VARIANT_ID, 1, dev)
        time.sleep(0.1)
        dev = dev["device"]()
        if (rslt == BME68X_OK):
            dev["variant_id"] = dev["intf_rslt"][5]
        return rslt
