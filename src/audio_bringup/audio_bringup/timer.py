#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
timer.py: Swithc audio sending on and off regularly

"""

import subprocess
import time

def set_param(node_name, param_name, param_value):
    param_pid = subprocess.Popen(['ros2', 'param', 'set', node_name, param_name, param_value], stdout=subprocess.PIPE)
    print('waiting to set params:', param_name, param_value)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    if out_string == "Set parameter successful":
        return True
    else:
        print("set_param error:", out_string)
        return False


if __name__ == "__main__":
    import time

    period = 5 # seconds
    switch = True
    try:
        while 1:
            set_param('/gateway', 'send_audio_enable', str(int(switch)))
            switch = not switch
            time.sleep(period)
    except KeyboardInterrupt:
        set_param('/gateway', 'send_audio_enable', str(1))
