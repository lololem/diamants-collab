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
Tool for manipulating and adding data to the automatically generated reports.
"""
import numpy as np
from numpy.polynomial import polynomial as P
from scipy.interpolate import CubicSpline, BSpline, splrep


# from model_payload import ResidualsPayload


class DataHelper:
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def generate_data(data: dict[str, np.ndarray], 
                      event: str, 
                      info: dict[str, str | int | float]) -> dict[str, np.ndarray]:
        source = data[event].get(info.get("source", None), None)
        t = data[event]["timestamp"]
        t_fit = data[event].get("fitTimestamp", None)
        
        if info.get("derivative", 0) < 0:
            raise ValueError("Derivative must be greater than or equal to 0")

        if info["type"] == "linspace":
            data_new = DataHelper.generate_data_linspace(source, info["step"])
        elif info["type"] == "poly":
            data_new = DataHelper.generate_data_poly(t, source, t_fit, info)
        elif info["type"] == "cs":
            data_new = DataHelper.generate_data_cs(t, source, t_fit, info)
        elif info["type"] == "bs":
            data_new = DataHelper.generate_data_bs(t, source, t_fit, info)
        elif info["type"] == "custom":
            data_new = DataHelper.generate_data_custom(data[event], info["target"])
        else:
            raise NotImplementedError

        # exit 1: add each vector of the custom data list iteratively to the data dictionary and return
        if isinstance(info["target"], list):
            dict_new = {}
            for i, target in enumerate(info["target"]):
                dict_new[target] = data_new[i]
            
            return dict_new

        # exit 2: add single vector to dictionary and return
        return {info["target"]: data_new}

    @staticmethod
    def generate_data_linspace(x: np.ndarray, step: int) -> np.ndarray:
        return np.arange(x[0], x[-1], step)

    @staticmethod
    def generate_data_poly(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, info: dict[str, str | int | float]) -> np.ndarray:
        p = P.Polynomial.fit(x, y,  info["degree"])
        p = p.deriv(info["derivative"])

        if not info.get("original_length", False):
            return p(x_fit)

        return p(x)
    
    @staticmethod
    def generate_data_cs(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, info: dict[str, str | int | float]) -> np.ndarray:
        cs = CubicSpline(x, y)

        if not info.get("original_length", False):
            return cs(x_fit, info["derivative"])
        
        return cs(x, info["derivative"])
    
    @staticmethod
    def generate_data_bs(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, info: dict[str, str | int | float]) -> np.ndarray:
        tck = splrep(x, y, s=info["smoothing"])
        bs = BSpline(*tck)

        if not info.get("original_length", False):
            return bs(x_fit, info["derivative"])
        
        return bs(x, info["derivative"])  
    
    @staticmethod
    def generate_data_custom(data: dict[str, np.ndarray], target_list: list[str]) -> list[np.ndarray]:
        pass
        # init objects for computing custom data (residuals, state errors, etc.)
        # res_payload = ResidualsPayload(data)

        # check and generate target for custom data
        # custom_data = []
        # for target in target_list:
        #     if target == "error.px":
        #         custom_data.append(res_payload.get_error_payload_position_x())
        #     elif target == "error.py":
        #         custom_data.append(res_payload.get_error_payload_position_y())
        #     elif target == "error.pz":
        #         custom_data.append(res_payload.get_error_payload_position_z())
        #     elif target == "error.pvx":
        #         custom_data.append(res_payload.get_error_payload_velocity_x())
        #     elif target == "error.pvy":
        #         custom_data.append(res_payload.get_error_payload_velocity_x())
        #     elif target == "error.pvz":
        #         custom_data.append(res_payload.get_error_payload_velocity_x())
        #     elif target == "error.cpx":
        #         custom_data.append(res_payload.get_error_cable_unit_vector_x())
        #     elif target == "error.cpy":
        #         custom_data.append(res_payload.get_error_cable_unit_vector_y())
        #     elif target == "error.cpz":
        #         custom_data.append(res_payload.get_error_cable_unit_vector_z())
        #     elif target == "error.pwx":
        #         custom_data.append(res_payload.get_error_payload_angular_velocity_x())
        #     elif target == "error.pwy": 
        #         custom_data.append(res_payload.get_error_payload_angular_velocity_y())
        #     elif target == "error.pwz":
        #         custom_data.append(res_payload.get_error_payload_angular_velocity_z())
        #     elif target == "error.rpyx":
        #         custom_data.append(res_payload.get_error_uav_orientation_x())
        #     elif target == "error.rpyy":
        #         custom_data.append(res_payload.get_error_uav_orientation_y())
        #     elif target == "error.rpyz":
        #         custom_data.append(res_payload.get_error_uav_orientation_z())
        #     elif target == "error.wx":
        #         custom_data.append(res_payload.get_error_uav_angular_velocity_x())
        #     elif target == "error.wy":
        #         custom_data.append(res_payload.get_error_uav_angular_velocity_y())
        #     elif target == "error.wz":
        #         custom_data.append(res_payload.get_error_uav_angular_velocity_z())
        #     elif target == "residual.f":
        #         custom_data.append(res_payload.get_residual_force())
        #     elif target == "residual.tx":
        #         custom_data.append(res_payload.get_residual_torque_x())
        #     elif target == "residual.ty":
        #         custom_data.append(res_payload.get_residual_torque_y())
        #     elif target == "residual.tz":
        #         custom_data.append(res_payload.get_residual_torque_z())

        # return custom_data
    
    
if __name__ == "__main__":
    pass
    # small test
    # data = {"event": {
    #         "timestamp": np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
    #         "source": np.array([10, 20, 15, 30, 15, 10, 5, 10, 15, 20])}}

    # info = {"type": "poly",
    #         "degree": 100,
    #         "derivative": 1,
    #         "source": "source",
    #         "target": "target"}

    # target, generated_data = DataHelper.generate_data(data, "event", info)
    # plt.plot(data["event"]["timestamp"], data["event"]["source"], label="source")
    # plt.plot(data["event"]["timestamp"], generated_data, label="generated")
    # plt.legend()
    # plt.show()

    # another small test
    # x = np.arange(10)
    # y = np.sin(x)
    # cs = CubicSpline(x, y)
    # xs = np.arange(-0.5, 9.6, 0.1)
    # fig, ax = plt.subplots(figsize=(6.5, 4))
    # ax.plot(x, y, 'o', label='data')
    # ax.plot(xs, np.sin(xs), label='true')
    # ax.plot(xs, cs(xs), label="S")
    # ax.plot(xs, cs.derivative(1), label="S'")
    # ax.plot(xs, cs.derivative(2), label="S''")
    # ax.plot(xs, cs.derivative(3), label="S'''")
    # ax.set_xlim(-0.5, 9.5)
    # ax.legend(loc='lower left', ncol=2)
    # plt.show()

    # print(cs.c.shape)
    # csder = cs.derivative(1)
    # print(csder.c.shape)