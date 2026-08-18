# Telemetry and hardware state of the STM32 Mower Board.

std_msgs/Header header

# Safety critical: True if the hardware kill switch is engaged.
bool kill_switch_engaged

# Power rail status feedback
bool is_12v_enabled
bool is_5v_enabled
bool is_jetson_power_enabled

# Future-proofing: If ADC is implemented later for battery monitoring
# float32 battery_voltage_v
# float32 battery_current_a
