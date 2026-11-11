# Thrusters and kill switches. Topic: /mower/motors

builtin_interfaces/Time stamp
int8[8] thrust_pct            # applied speed per thruster, -100..100 (0 = stopped)
bool lcd_kill_active          # LCD kill switch pulled (motors + 5V off)
bool enclosure_kill_active    # enclosure kill switch pulled (motors + 5V + 12V off)
bool kill_since_last_cmd      # a kill happened since the last /motor_cmd
uint32 last_cmd_age_ms        # time since last /motor_cmd, 4294967295 = none yet
