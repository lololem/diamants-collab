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

from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import rosbag2_py
import csv

class McapHandler:
    def __init__(self):
        self.takeoff_time = None

    def read_messages(self, input_bag: str):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f"topic {topic_name} not in bag")

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        del reader
    
    def write_mcap_to_csv(self, inputbag:str, outputfile:str):
        '''A method which translates an .mcap rosbag file format to a .csv file. 
        Also modifies the timestamp to start at 0.0 instead of the wall time.
        Only written to translate the /tf topic but could easily be extended to other topics'''
        t_start_bag = None #this is the timestamp of the first message we read in the bag (doesn't mean it's exactly the start time of the bag but close enough ?)
        try:
            print("Translating .mcap to .csv")
            f = open(outputfile, 'w+')
            writer = csv.writer(f)
            writer.writerow(["# t"," x"," y"," z"])
            for topic, msg, timestamp in self.read_messages(inputbag):
                if topic =="/tf":
                    if t_start_bag == None:
                        t_start_bag = msg.transforms[0].header.stamp.sec + msg.transforms[0].header.stamp.nanosec *10**(-9) 
                    t = msg.transforms[0].header.stamp.sec + msg.transforms[0].header.stamp.nanosec *10**(-9) -t_start_bag
                    writer.writerow([t, msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z])
                if topic == "/rosout":  #and t_start_bag != None :
                    if msg.name == "crazyflie_server" and msg.function == "takeoff":
                        t = msg.stamp.sec + msg.stamp.nanosec *10**(-9) - t_start_bag
                        self.takeoff_time = t
                    #the takeoff message in simulation has a sligthly different name than IRL, so we need this to record the sim takeoff time
                    #BUT for some unknown reason the sim tests seem to work perfectly without even recording takeoff and adjusting the offset
                    #so I will leave this commented here just in case the sim tests have to be worked on
                    # if msg.name == "crazyflie_server" and msg.function == "_takeoff_callback":
                    #     t = msg.stamp.sec + msg.stamp.nanosec *10**(-9)
                    #     self.takeoff_time = t
                    #     print(f"takeoff at {self.takeoff_time}")

            #write the "takeoff command" time as a comment on the last line
            writer.writerow([f"### takeoff time : {self.takeoff_time}"])
            f.close()
        except FileNotFoundError:
            print(f"McapHandler : file {outputfile} not found")
            exit(1)




if __name__ == "__main__":

    #command line utility 

    from argparse import ArgumentParser, Namespace
    parser = ArgumentParser(description="Translates the /tf topic of an .mcap rosbag file format to a .csv file")
    parser.add_argument("inputbag", type=str, help="The .mcap rosbag file to be translated")
    parser.add_argument("outputfile", type=str, help="Output csv file that has to be created/overwritten")
    args:Namespace = parser.parse_args()

    translator =  McapHandler()
    translator.write_mcap_to_csv(args.inputbag,args.outputfile)