# DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
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

import os

def log_fusion_weighted_inputs(tick, merged_data, merged_validated_data, consensus_data, union_data, fusion_weighted, fusion_weighted_smoothed):
    os.makedirs('logs', exist_ok=True)
    with open('logs/map_merger_weighted_inputs_debug.log', 'a') as debug_log:
        debug_log.write(f"Tick {tick}\n")
        debug_log.write(f"merged_data[:20]: {merged_data[:20]}\n")
        debug_log.write(f"merged_validated_data[:20]: {merged_validated_data[:20]}\n")
        debug_log.write(f"consensus_data[:20]: {consensus_data[:20]}\n")
        debug_log.write(f"union_data[:20]: {union_data[:20]}\n")
        debug_log.write(f"fusion_weighted[:20]: {fusion_weighted[:20]}\n")
        if fusion_weighted_smoothed is not None:
            debug_log.write(f"fusion_weighted_smoothed[:20]: {fusion_weighted_smoothed[:20]}\n\n")
        else:
            debug_log.write("fusion_weighted_smoothed: None (pas assez d'historique)\n\n")
