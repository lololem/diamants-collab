# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# -*- coding: utf-8 -*-
"""
Tool for yaml-based automatic report generation from logged data of the crazyflie.
"""

# attidtue best: 22

import SDplotting.cfusdlog
import matplotlib.pyplot as plt
import os
import sys
import yaml
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np


import SDplotting.data_helper


def file_guard(pdf_path):
    msg = None
    if os.path.exists(pdf_path):
        msg = input("file already exists, overwrite? [y/n]: ")
        if msg == "n":
            print("exiting...")
            sys.exit(0)
        elif msg == "y":
            print("overwriting...")
            os.remove(pdf_path)
        else:
            print("invalid msg...")
            file_guard(pdf_path)

    return


def process_data(data, settings):
    print("...processing data")
    # convert units
    event = settings["event_name"]
    
    
    # if we have a list of events we need to iterate over each individually
    if type(event) == list :        
        for one_event in event:
            if settings["convert_units"]:
                for key, value in settings["convert_units"].items():
                    if key in data[one_event].keys():
                        data[one_event][key] = data[one_event][key] * value                       
            # shift time vector to start at 0
            data[one_event]["timestamp"] = (data[one_event]["timestamp"] - data[one_event]['timestamp'][0])
            
   

            # crop data
            start_time = settings["start_time"]
            end_time = settings["end_time"]
            
            t = data[one_event]["timestamp"]
            if start_time is not None:
                for key, value in data[one_event].items():
                    data[one_event][key] = value[t >= start_time]

            t = data[one_event]["timestamp"]
            if end_time is not None:
                for key, value in data[one_event].items():
                    data[one_event][key] = value[t <= end_time]

            # add additional data to the data dictionary
            # add_data(data, settings)

            # print(data[one_event].keys())
            # print(data[one_event].items())
    
    
     # else:   #if only one event
    #     if settings["convert_units"]:
    #         for key, value in settings["convert_units"].items():
    #             data[event][key] = data[event][key] * value
    #    # shift time vector to start at 0
    #     data[event]["timestamp"] = (data[event]["timestamp"] - data[event]['timestamp'][0])
        
    #     # crop data
    #     start_time = settings["start_time"]
    #     end_time = settings["end_time"]
        
    #     t = data[event]["timestamp"]
    #     if start_time is not None:
    #         for key, value in data[event].items():
    #             data[event][key] = value[t >= start_time]

    #     t = data[event]["timestamp"]
    #     if end_time is not None:
    #         for key, value in data[event].items():
    #             data[event][key] = value[t <= end_time]

        # add additional data to the data dictionary
        # add_data(data, settings)

        # print(data[event].keys())
        # print(data[event].items())

    return data


def add_data(data, settings):
    event = settings["event_name"]
    print("...adding data")
    
    for info in settings["additional_data"]:
        # print(f"found target: {info['target']}")
        dict_new = data_helper.DataHelper.generate_data(data, event, info)
        data[event].update(dict_new)
        print(f">>> added data: {info['type']} -> {list(dict_new.keys())}")
        # print(f">>> data shape: {data_new.shape}")

    print("...done adding data")


def create_figures(data_usd, settings, logfile=None, out=None, ros2_ws=None, experiment=None):
    debug_all = False
    debug = False
    debug_figure_number = 20 # Residual Torques
    # debug_figure_number = 4 # UAV angles
    # debug_figure_number = 13 # payload position error
    # debug_figure_number = 6 # payload positions
    # debug_figure_number = 7 # payload velocities

    if logfile != None :
        log_path = logfile
    else:   #choose default log file path given in settings.yaml if no specific path given
        log_path = os.path.join(settings["data_dir"], settings['data_file'])
    
    print("log file: {}".format(log_path))

    data_processed = process_data(data_usd, settings)

    # create a PDF to save the figures
    if out!= None:
        pdf_path = out
    else:   #choose default pdf path in settings.yaml if no specifif one given
        pdf_path =  os.path.join(settings["output_dir"], settings['data_file']) + ".pdf"
    print("output path: {}".format(pdf_path))

    # check if user wants to overwrite the report file
    file_guard(pdf_path)

    pdf_pages = PdfPages(pdf_path)

    # create the title page
    title_text_settings = f"Settings:\n"
    for setting in settings["title_settings"]:
        title_text_settings += f"    {setting}: {settings[setting]}\n"

    # read the parameters from the info file
    info_path = os.path.join(settings['info_dir'], settings["info_file"])
    if ros2_ws != None and experiment != None:
        info_path = str(ros2_ws) + f"/src/crazyswarm2/systemtests/SDplotting/info/info_{experiment}.yaml"
    print("... reading info file: {}".format(info_path))

    try:
        with open(info_path, "r") as f:
            info = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"(plot.py) File not found: {info_path}")
        exit(1)

    title_text_parameters = f"Parameters:\n"
    for key, value in info.items():
        title_text_parameters += f"    {key}: {value}\n"

    text = f"%% Report %%\n"
    title_text = text + "\n" + title_text_settings + "\n" + title_text_parameters + "\n" # + title_text_results
    fig = plt.figure(figsize=(5, 8))
    fig.text(0.1, 0.1, title_text, size=11)
    pdf_pages.savefig(fig)

    # create data plots for each event
    figures_max = settings.get("figures_max", None)  # set to None to plot all figures
    figure_count = 0
    for k, (event, data) in enumerate(data_processed.items()):
        if event in settings["event_name"]:
            print("processing event: {} ({})".format(event, k))

            # create a title text for each event
            t = f"%% Event {k}: {event} %%\n"
            fig = plt.figure(figsize=(5, 5))
            fig.text(0.1, 0.1, title_text, size=11)
            pdf_pages.savefig(fig)

            # create a new figure for each value in the data dictionary
            figures_key = f"figures_{event}"
            for figure_info in settings[figures_key]:
                if figures_max is not None and figure_count >= figures_max:
                    break

                title = figure_info["title"]
                figure_type = figure_info["type"]
                x_label = figure_info.get("x_label", None)
                y_label = figure_info.get("y_label", None)
                z_label = figure_info.get("z_label", None)
                structure = figure_info["structure"]
                structure_length = len(structure)
                
                if figure_type == "2d subplots":
                    fig, ax = plt.subplots(structure_length, 1)

                    if structure_length == 1:
                        ax = [ax]
                    
                    # iterate over every subplot in the figure
                    for i, obj in enumerate(structure):
                        n_x = len(obj["x_axis"])
                        n_y = len(obj["y_axis"])
                        n_leg = len(obj["legend"])

                        if n_x != n_y != n_leg:
                            raise ValueError("Please specify the same number of x and y signals and legends")
                        
                        # iterate over every plot in the respective subplot
                        for j in range(n_x):
                            x = obj["x_axis"][j]
                            y = obj["y_axis"][j]
                            
                            # print(obj["legend"][j], bool(obj["legend"][j]))
                            if figure_info["marker"] == "line":
                                ax[i].plot(data[x], data[y], label=obj["legend"][j], **figure_info["marker_kwargs"])
                            elif figure_info["marker"] == "scatter":
                                ax[i].scatter(data[x], data[y], label=obj["legend"][j], **figure_info["marker_kwargs"])
                            else:
                                raise ValueError("Invalid marker")

                            ax[i].set_xlabel(obj["x_label"])
                            ax[i].set_ylabel(obj["y_label"])
                            if obj["legend"][j]:
                                ax[i].legend(loc="lower left", fontsize=5)
                            ax[i].grid(True)

                # DEPRECATED
                # if figure_type == "2d single":
                #     fig, ax = plt.subplots()
                    
                #     # iterate over every subplot
                #     for obj in structure:
                #         ax.plot(data[obj["x_axis"]], 
                #             data[obj["y_axis"]], 
                #             label=obj["legend"], 
                #             linewidth=0.5)
                #         ax.set_xlabel(obj["x_label"])
                #         ax.set_ylabel(obj["y_label"])
                #         ax.legend(loc="lower left", fontsize=5)
                #         ax.grid(True)

                if figure_type == "3d":
                    fig = plt.figure()
                    ax = fig.add_subplot(projection='3d')

                    y_label = figure_info["y_label"]
                    
                    # iterate over every subplot
                    for i, obj in enumerate(structure):
                        ax.plot(data[obj[0]],
                                data[obj[1]],
                                data[obj[2]], 
                                label=obj[3], 
                                linewidth=0.5)
                        
                        ax.set_xlim(min(data[obj[0]])-0.1*min(data[obj[0]]), 
                                    max(data[obj[0]])+0.1*max(data[obj[0]]))
                        ax.set_ylim(min(data[obj[1]])-0.1*min(data[obj[1]]),
                                    max(data[obj[1]])+0.1*max(data[obj[1]]))
                        ax.set_zlim(min(data[obj[2]])-0.1*min(data[obj[2]]),
                                    max(data[obj[2]])+0.1*max(data[obj[2]]))

                    ax.set_xlabel(x_label)
                    ax.set_ylabel(y_label)
                    ax.set_zlabel(z_label)
                    ax.legend(loc="lower left", fontsize=5)
                    ax.grid(True)

                # show plot for debugging
                if debug and figure_count == debug_figure_number-1 or debug_all:
                    plt.show()

                fig.suptitle(title, fontsize=16)
                plt.tight_layout()
                
                # save the figure as a page in the PDF
                pdf_pages.savefig(fig)
                plt.close(fig)

                figure_count += 1
                status_text = ">>> created figure {}: {}".format(figure_count, title)
                print(status_text)

    pdf_pages.close()


def plot_SD_data(logfile=None, output = None, experiment=None, ros2_ws = None):

    # change the current working directory to the directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # load the plot settings
    settings_file = "settings.yaml"
    with open(settings_file, 'r') as f:
        settings = yaml.load(f, Loader=yaml.FullLoader)
    
    if experiment != None:
        settings["data_file"] = experiment
        settings["info_file"] = "info_" + experiment + ".csv"

    # decode binary log data
    if logfile != None :
        path = logfile
    else:   #choose default path given in settings.yaml
        path = os.path.join(settings["data_dir"], settings['data_file'])
    print(f"Processing {path}...")
    data_usd = SDplotting.cfusdlog.decode(path)

    # create the figures
    print("...creating figures")
    create_figures(data_usd, settings, out=output, ros2_ws=ros2_ws, experiment=experiment)
    print("...done creating figures")

    
if __name__ == "__main__":
    plot_SD_data()