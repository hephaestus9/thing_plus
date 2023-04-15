import dearpygui.dearpygui as dpg

temp_f_dataY = []
temp_c_dataY = []
humidity_dataY = []
temp_dataX = []


def show_env_window():
    if not dpg.does_item_exist("env_window"):
        env_window()
    else:
        dpg.delete_item("env_window")
        env_window()


def env_window():
    with dpg.window(label="Environmental Data", tag="env_window", pos=(0,450), width=1200, height=425, no_collapse=True):    
        dpg.add_text(" ", tag="current_temp_val")
        with dpg.plot(label="Temperature", tag="temp_plot", height=350, width=-1):
            # optionally create legend
            #dpg.add_plot_legend()
            # REQUIRED: create x and y axes
            dpg.add_plot_axis(dpg.mvXAxis, label="Data Point", tag="temp_f_x_axis")

            # create y axis 1
            dpg.add_plot_axis(dpg.mvYAxis, label="Temperature (F)", tag="temp_f_plot_axis")
            dpg.add_line_series(temp_dataX, temp_f_dataY, tag="temp_plot_data", label="Temperature (F)", parent=dpg.last_item())
            #dpg.add_line_series(test_off_dataX, test_off_dataY, tag="test_off_plot_data", label="Time")"""

            # create y axis 2
            dpg.add_plot_axis(dpg.mvYAxis, label="Humidity (rH)", tag="humidity_plot_axis")
            dpg.add_line_series(temp_dataX, humidity_dataY, label="Humidity (rH)", tag="humidity_plot_data", parent=dpg.last_item())