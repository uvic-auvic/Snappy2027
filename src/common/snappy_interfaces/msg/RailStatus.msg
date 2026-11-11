# Aux power rails. Topic: /mower/rails

builtin_interfaces/Time stamp
bool v5_on                    # output enabled now
bool v5_requested             # last Jetson request (reset to on at kill re-arm)
bool v5_killed                # held off by a kill switch
bool v5_power_good            # INA260 reads > 4.5 V
bool v12_on
bool v12_requested
bool v12_killed
bool v12_power_good           # INA260 reads > 10.8 V
