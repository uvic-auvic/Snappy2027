# Commands up to 8 BlueRobotics T200 thrusters connected to the Mower Board.

# Header provides timestamp for latency tracking and EKF synchronization.
# frame_id should generally be "base_link" or empty for raw hardware commands.
std_msgs/Header header

# Bitmask indicating which thrusters to update. 
# Bit 0 (1) = Thruster 1 ... Bit 7 (128) = Thruster 8
# This saves bandwidth over UART by allowing partial updates.
uint8 thruster_mask

# Target thrust percentages for each of the 8 thrusters.
# Range: [-100, 100]. 0 is stop, -100 is max reverse, 100 is max forward.
int8[8] thrust_pct
