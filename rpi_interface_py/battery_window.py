import dearpygui.dearpygui as dpg

battery_voltage_dataX = []
battery_voltage_dataY = []


def show_battery_window():
    if not dpg.does_item_exist("battery_window"):
        battery_window()
    else:
        dpg.delete_item("battery_window")
        battery_window()


def battery_window():
    with dpg.window(label="Battery", tag="battery_window", pos=(0,450), width=1200, height=425, no_collapse=True):    
        dpg.add_text(" ", tag="pcnt_charge_val")
        with dpg.plot(label="Battery Voltage", tag="battery_voltage_plot", height=350, width=-1):
            # optionally create legend
            #dpg.add_plot_legend()
            # REQUIRED: create x and y axes
            dpg.add_plot_axis(dpg.mvXAxis, label="Data Point", tag="battery_voltage_x_axis")
            with dpg.plot_axis(dpg.mvYAxis, label="Voltage (V)", tag="battery_voltage_plot_axis"):
                # series belong to a y axis
                dpg.add_line_series(battery_voltage_dataX, battery_voltage_dataY, tag="battery_voltage_plot_data", label="Voltage (V)")
                #dpg.add_line_series(test_off_dataX, test_off_dataY, tag="test_off_plot_data", label="Time")"""