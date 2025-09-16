#!/usr/bin/env python3

"""
A flash utility script to flash crazyflies with the latest or custom firmware

    2024 - K. N. McGuire (Bitcraze AB)
"""

import rclpy
from rclpy.node import Node
import cflib.crtp  # noqa
from cflib.bootloader import Bootloader, Target
from cflib.bootloader.boottypes import BootVersion
import argparse
import os

class Flash(Node):
    def __init__(self, uri, file_name):
        super().__init__('flash')
    
        self.get_logger().info(f"Flashing {uri} with {file_name}")

        base_file_name = os.path.basename(file_name)

        if base_file_name.endswith("zip") and base_file_name.startswith("firmware-cf2"):
            targets = []
        elif base_file_name.endswith("bin") and base_file_name.startswith("cf2"):
            targets = [Target("cf2", 'stm32', 'fw', [], [])]
        else:
            self.get_logger().error(f"Unsupported file type or name. Only cf2*.bin or firmware-cf2*.zip supported")
            return

        bl = Bootloader(uri)
        try:
            bl.flash_full(None, file_name, True, targets)
        except Exception as e:
            self.get_logger().error(f"Failed to flash, Error {str(e)}")
        finally:
            if bl:
                bl.close()

        return

def main(args=None):
    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="This is a sample script")
    parser.add_argument('--uri', type=str, default="radio://0/80/2M/E7E7E7E7E7", help='unique resource identifier')
    parser.add_argument('--file_name', type=str, default="cf2.bin", help='')

    # Parse the arguments
    args = parser.parse_args()

    print("URI: ", args.uri)
    flash_node = Flash(args.uri, args.file_name)

    flash_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()