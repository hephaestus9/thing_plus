import BME688.bme688_defs as bme688_defs
import BME688.bme688_api as bme688
import time
import datetime


class bme688_interface():
    def __init__(self, board, addr=0x77, initSettings=True, debug=False):
        self.board = board
        self.addr = addr
        self.board.set_pin_mode_i2c()
        self.success = True
        self.debug = debug

        #/** Value returned by remainingReadingMillis indicating no asynchronous
        #* reading has been initiated by beginReading. **/
        self.reading_not_started = -1
        #/** Value returned by remainingReadingMillis indicating asynchronous reading
        #* is complete and calling endReading will not block. **/
        self.reading_complete = 0

        #/** Temperature (Celsius) assigned after calling performReading() or
        #* endReading() **/
        self.temperature = 0.0
        #/** Pressure (Pascals) assigned after calling performReading() or endReading()
        #* **/
        self.pressure = 0.0
        #/** Humidity (RH %) assigned after calling performReading() or endReading()
        #* **/
        self.humidity = 0.0
        #/** Gas resistor (ohms) assigned after calling performReading() or
        #* endReading() **/
        self.gas_resistance = 0.0

        #self._sensorID
        self._meas_start = 0
        self._meas_period = 0

        self.reg_data = None

        self.gas_sensor = bme688_defs.bme68x_dev
        self.gas_conf = bme688_defs.bme68x_conf
        self.gas_heatr_conf = bme688_defs.bme68x_heatr_conf

        self.gas_sensor["chip_id"] = self.addr
        self.gas_sensor["intf"] = "i2c"
        self.gas_sensor["intf_ptr"] = "i2c"
        self.gas_sensor["read"] = self.i2c_read
        self.gas_sensor["write"] = self.i2c_write
        self.gas_sensor["amb_temp"] = 25  #/* The ambient temperature in deg C is used for
                                  #   defining the heater temperature */
        self.gas_sensor["delay_us"] = self.delay_usec
        self.gas_sensor["device"] = self.get_device
        self.gas_sensor["set_device"] = self.set_device
    
        self.dev = bme688.BME68x()
        rslt = self.dev.begin(self.gas_sensor)

        if self.debug:
            print("Init Result: " + str(rslt))

        if (rslt != bme688_defs.BME68X_OK):
            self.success = False
        
        if self.debug:
            print("T1 = " + str(self.gas_sensor["calib"]["par_t1"]))
            print("T2 = " + str(self.gas_sensor["calib"]["par_t2"]))
            print("T3 = " + str(self.gas_sensor["calib"]["par_t3"]))
            print("P1 = " + str(self.gas_sensor["calib"]["par_p1"]))
            print("P2 = " + str(self.gas_sensor["calib"]["par_p2"]))
            print("P3 = " + str(self.gas_sensor["calib"]["par_p3"]))
            print("P4 = " + str(self.gas_sensor["calib"]["par_p4"]))
            print("P5 = " + str(self.gas_sensor["calib"]["par_p5"]))
            print("P6 = " + str(self.gas_sensor["calib"]["par_p6"]))
            print("P7 = " + str(self.gas_sensor["calib"]["par_p7"]))
            print("P8 = " + str(self.gas_sensor["calib"]["par_p8"]))
            print("P9 = " + str(self.gas_sensor["calib"]["par_p9"]))
            print("P10 = " + str(self.gas_sensor["calib"]["par_p10"]))
            print("H1 = " + str(self.gas_sensor["calib"]["par_h1"]))
            print("H2 = " + str(self.gas_sensor["calib"]["par_h2"]))
            print("H3 = " + str(self.gas_sensor["calib"]["par_h3"]))
            print("H4 = " + str(self.gas_sensor["calib"]["par_h4"]))
            print("H5 = " + str(self.gas_sensor["calib"]["par_h5"]))
            print("H6 = " + str(self.gas_sensor["calib"]["par_h6"]))
            print("H7 = " + str(self.gas_sensor["calib"]["par_h7"]))
            print("G1 = " + str(self.gas_sensor["calib"]["par_gh1"]))
            print("G2 = " + str(self.gas_sensor["calib"]["par_gh2"]))
            print("G3 = " + str(self.gas_sensor["calib"]["par_gh3"]))
            print("Heat Range = " + str(self.gas_sensor["calib"]["res_heat_range"]))
            print("Heat Val = " + str(self.gas_sensor["calib"]["res_heat_val"]))
            print("SW Error = " + str(self.gas_sensor["calib"]["range_sw_err"]))

        if initSettings:
            self.setIIRFilterSize(bme688_defs.BME68X_FILTER_SIZE_3)
            self.setODR(bme688_defs.BME68X_ODR_NONE)
            self.setHumidityOversampling(bme688_defs.BME68X_OS_2X)
            self.setPressureOversampling(bme688_defs.BME68X_OS_4X)
            self.setTemperatureOversampling(bme688_defs.BME68X_OS_8X)
            self.setGasHeater(320, 150) #// 320*C for 150 ms
        else:
            self.setGasHeater(0, 0)
        
        #// don't do anything till we request a reading
        rslt = self.dev.bme68x_set_op_mode(bme688_defs.BME68X_FORCED_MODE, self.gas_sensor)

        if self.debug:
            print("Opmode Result: " + str(rslt))
 

        if (rslt != bme688_defs.BME68X_OK):
            self.success = False
    
    def i2c_write(self, reg_addr, reg_data, dev):
        self.board.i2c_write(dev["chip_addr"], [reg_addr, int(reg_data)])
        self.i2c_read(reg_addr, 1, dev)
        return 0

    def i2c_read(self, reg_addr, rlen, dev):
        self.board.i2c_read(dev["chip_addr"], reg_addr, rlen, self.read_callback)
        return 0
    
    def read_callback(self, data):
        #print(data)
        self.gas_sensor["intf_rslt"] = data
        
    def get_device(self):
        return self.gas_sensor
    
    def set_device(self, dev):
        self.gas_sensor = dev

    def delay_usec(self, us):
        time.sleep(us/1000/1000)

    def readTemperature(self):
        return self.temperature

    def readPressure(self):
        return self.pressure

    def readHumidity(self):
        return self.humidity

    def readGas(self):
        return self.gas_resistance

    def readAltitude(self, seaLevel):
        #// Equation taken from BMP180 datasheet (page 16):
        #//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

        #// Note that using the equation from wikipedia can give bad results
        #// at high altitude. See this thread for more information:
        #//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

        atmospheric = self.readPressure() / 100.0
        return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903))

    def setTemperatureOversampling(self, os):
        if (os > bme688_defs.BME68X_OS_16X):
            return False

        self.gas_conf["os_temp"] = os

        rslt = self.dev.bme68x_set_conf(self.gas_conf, self.gas_sensor)
        if self.debug:
            print("SetConf Result: " + str(rslt))
        return rslt == 0

    def setPressureOversampling(self, os):
        if (os > bme688_defs.BME68X_OS_16X):
            return False

        self.gas_conf["os_pres"] = os

        rslt = self.dev.bme68x_set_conf(self.gas_conf, self.gas_sensor)
        if self.debug:
            print("SetConf Result: " + str(rslt))
        return rslt == 0

    def setHumidityOversampling(self, os):
        if (os > bme688_defs.BME68X_OS_16X):
            return False

        self.gas_conf["os_hum"] = os

        rslt = self.dev.bme68x_set_conf(self.gas_conf, self.gas_sensor)
        if self.debug:
            print("SetConf Result: " + str(rslt))
        return rslt == 0

    def setIIRFilterSize(self, fs):
        if (fs > bme688_defs.BME68X_FILTER_SIZE_127):
            return False
        self.gas_conf["filter"] = fs

        rslt = self.dev.bme68x_set_conf(self.gas_conf, self.gas_sensor)
        if self.debug:
            print("SetConf Result: " + str(rslt))
        return rslt == 0

    def setGasHeater(self, heaterTemp, heaterTime):
        if ((heaterTemp == 0) or (heaterTime == 0)):
            self.gas_heatr_conf["enable"] = bme688_defs.BME68X_DISABLE
        else:
            self.gas_heatr_conf["enable"] = bme688_defs.BME68X_ENABLE
            self.gas_heatr_conf["heatr_temp"] = heaterTemp
            self.gas_heatr_conf["heatr_dur"] = heaterTime

        rslt = self.dev.bme68x_set_heatr_conf(bme688_defs.BME68X_FORCED_MODE, self.gas_heatr_conf, self.gas_sensor)
        if self.debug:
            print("SetHeaterConf Result: " + str(rslt))
        return rslt == 0

    def setODR(self, odr):
        if (odr > bme688_defs.BME68X_ODR_NONE):
            return False

        self.gas_conf["odr"] = odr

        rslt = self.dev.bme68x_set_conf(self.gas_conf, self.gas_sensor)
        if self.debug:
            print("SetConf Result: " + str(rslt))
        return rslt == 0

    #// Perform a reading in blocking mode.
    def performReading(self):
        return self.endReading()

    def beginReading(self):
        if (self._meas_start != 0):
            #/* A measurement is already in progress */
            return self._meas_start + self._meas_period;

        rslt = self.dev.bme68x_set_op_mode(bme688_defs.BME68X_FORCED_MODE, self.gas_sensor)
        if self.debug:
            print("Opmode Result: " + str(rslt))
        if (rslt != bme688_defs.BME68X_OK):
            return False

        #/* Calculate delay period in microseconds */
        delayus_period = self.dev.bme68x_get_meas_dur(bme688_defs.BME68X_FORCED_MODE, self.gas_conf, self.gas_sensor) + (self.gas_heatr_conf["heatr_dur"] * 1000)
        #// Serial.print("measure: ");
        #// Serial.println(bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf,
        #// &gas_sensor)); Serial.print("heater: ");
        #// Serial.println((uint32_t)gas_heatr_conf.heatr_dur * 1000);

        self._meas_start = datetime.datetime.now()
        self._meas_period = delayus_period #/ 1000

        return (self._meas_start.microsecond + self._meas_period) / 1000

    def endReading(self):
        meas_end = self.beginReading()
        
        if (meas_end == 0):
            return False

        remaining_millis = self.remainingReadingMillis()
    
        if (remaining_millis > 0):
            if self.debug:
                print("Waiting (ms) " + str(remaining_millis))
            time.sleep((remaining_millis * 2) / 1000) #/* Delay till the measurement is ready */

        self._meas_start = 0 #/* Allow new measurement to begin */
        self._meas_period = 0

        if self.debug:
            print("t_fine = " + str(self.gas_sensor["calib"]["t_fine"]))

        data = bme688_defs.bme68x_data
        n_fields = 0

        if self.debug:
            print("Getting sensor data")

        rslt = self.dev.bme68x_get_data(bme688_defs.BME68X_FORCED_MODE, data, n_fields, self.gas_sensor)
        if self.debug:
            print("GetData Result: " + str(rslt))

        if (rslt != bme688_defs.BME68X_OK):
            return False

        
        self.temperature = data["temperature"]
        self.humidity = data["humidity"]
        self.pressure = data["pressure"]

        if self.debug:
            print("data.status " + hex(data["status"]))

        if (data["status"] & (bme688_defs.BME68X_HEAT_STAB_MSK | bme688_defs.BME68X_GASM_VALID_MSK)):
            #// Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
            self.gas_resistance = data["gas_resistance"]
        else:
            self.gas_resistance = 0
            #// Serial.println("Gas reading unstable!");


        return True

    def remainingReadingMillis(self):
        time = datetime.datetime.now().microsecond
        return ((self._meas_start.microsecond + self._meas_period) / 1000) - (time / 1000)