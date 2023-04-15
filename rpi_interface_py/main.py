import dearpygui.dearpygui as dpg
import os
import sys
import numpy as np
import glob
import datetime
import serial
from telemetrix import telemetrix
import threading

from MAX1704X import max1704x
from BME688 import bme688_interface

from charts import *
from battery_window import *
from env_window import *

import db

def rootPath():
    if sys.platform == "darwin":
        return os.path.expanduser("~/Library/Application Support/SETHLANS/")
    elif sys.platform.startswith("win") or sys.platform.startswith("cli"):
        return os.path.join(os.environ['APPDATA'], "SETHLANS")
    else:
        return os.path.expanduser("~/.SETHLANS/")
    
def mc_interface_update(stop):
    start_t = datetime.datetime.now()
    sample_count = 1
    led_val = True
    while True: 
        if (datetime.datetime.now() - start_t).seconds >= 1:
            voltage = bat_dev.cellVoltage()
            prcnthr = bat_dev.chargeRate()
            cell_prcnt = bat_dev.cellPercent()
            if len(battery_voltage_dataX) > 3600:
                battery_voltage_dataX.pop(0)
                battery_voltage_dataY.pop(0)
                temp_f_dataY.pop(0)
                temp_c_dataY.pop(0)
                humidity_dataY.pop(0)
                temp_dataX.pop(0)
            
            battery_voltage_dataX.append(sample_count)
            battery_voltage_dataY.append(voltage)
            
            env_dev.performReading()
            temp = env_dev.readTemperature()
            humid = env_dev.readHumidity()
            gas = env_dev.readGas()

            temp_f_dataY.append(round((temp * (9/5)) + 32, 2))
            temp_c_dataY.append(round(temp, 2))
            humidity_dataY.append(round(humid, 2))
            temp_dataX.append(sample_count)

            #print(round(humid, 2), round(gas, 2))
            #print(prcnthr)
            #print(cell_prcnt)
            if dpg.does_item_exist("battery_window"):
                dpg.configure_item("pcnt_charge_val", default_value=str(round(cell_prcnt, 2)) + "% Charge")
            start_t = datetime.datetime.now()
            sample_count += 1
            if led_val:
                board.set_ws2812(0, 0, 155)
            else:
                board.set_ws2812(155, 0, 0)
                
            led_val = not led_val
            
        if exit_flag: 
            break

exit_flag = False

    
com_ports = []

def get_serial_ports():
        """
            Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        com_ports = ["Select COM Port"]
        for ser_port in ports:
            try:
                ser = serial.Serial(ser_port)
                ser.close()
                com_ports.append(ser_port)
            except (OSError, serial.SerialException):
                pass
        com_ports.append("Refresh")
        return com_ports

def com_port_select(sender, app_data, user_data):
    print(f"sender: {sender}, \t app_data: {app_data}, \t user_data: {user_data}")
    if app_data == "Refresh":
        com_ports = get_serial_ports()
        dpg.configure_item("com_port_select", items=com_ports)
        dpg.configure_item("com_port_select", default_value="Select COM Port")
    elif app_data != "Select COM Port":
        selected_com_port = app_data
    else:
        pass

def open_file_dialog():
    with dpg.file_dialog(label="Open Data File", width=800, height=400, file_count=1):
        dpg.add_file_extension(".dat")


com_ports = get_serial_ports()
rp2040_db = db.Db(os.path.join(rootPath(), 'rp2040.db'))
try:
    board = telemetrix.Telemetrix()
except:
    board = None

if board is not None:
    bat_dev = max1704x.MAX17048(board)
    bat_dev.begin()
    env_dev = bme688_interface.bme688_interface(board, debug=False)
    t = threading.Thread(target = mc_interface_update, args =(lambda : exit_flag, )) 
    t.daemon = True
    t.start()
else:
    t = None 

dpg.create_context()
dpg.create_viewport(title='RP2040 Interface', width=600, height=700)
dpg.set_viewport_small_icon(os. getcwd() + "\\icons8-valve-wanicon-lineal-96.ico")
dpg.set_viewport_large_icon(os. getcwd() + "\\icons8-valve-wanicon-lineal-128.ico")


data_count = 0
save_path = None
            

with dpg.window(label="MainWindow", tag="mainWindow", pos=(0,0), width=584, height=100, no_close=True, no_move=True, no_resize=True, no_title_bar=True):  
    
    def _log(sender, app_data, user_data):
        print(f"sender: {sender}, \t app_data: {app_data}, \t user_data: {user_data}")

    with dpg.menu_bar():
        with dpg.menu(label="Tools"):
            dpg.add_menu_item(label="Datalogging")
            dpg.add_separator()
            dpg.add_menu_item(label="Battery", callback=show_battery_window)
            dpg.add_menu_item(label="Environmental Sensors", callback=show_env_window)
            dpg.add_menu_item(label="Analog Input")
            dpg.add_menu_item(label="Digital I/O")

    #dpg.add_combo(com_ports, tag="com_port_select", label="", default_value=com_ports[0])

dpg.setup_dearpygui()
dpg.show_viewport()
dpg.set_primary_window("mainWindow", True)
while dpg.is_dearpygui_running():
    try:
        if dpg.does_item_exist("battery_window"):
            dpg.configure_item("battery_voltage_plot_data", x=battery_voltage_dataX)
            dpg.configure_item("battery_voltage_plot_data", y=battery_voltage_dataY)
            dpg.fit_axis_data("battery_voltage_x_axis")
            dpg.fit_axis_data("battery_voltage_plot_axis")
        if dpg.does_item_exist("env_window"):
            dpg.configure_item("temp_plot_data", y=temp_f_dataY)
            dpg.configure_item("temp_plot_data", x=temp_dataX)
            dpg.configure_item("humidity_plot_data", y=humidity_dataY)
            dpg.configure_item("humidity_plot_data", x=temp_dataX)
            dpg.fit_axis_data("temp_f_x_axis")
            #dpg.fit_axis_data("temp_f_plot_axis")
        dpg.render_dearpygui_frame()
    except Exception as e:
        print(e)
exit_flag = True
if t is not None:
    t.join() 
dpg.destroy_context()