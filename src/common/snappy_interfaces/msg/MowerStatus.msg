# Snapshot of all sectors, returned by /mower/status_request.
# sector_mask (StatusRequest SECTOR_* bits) says which sectors are filled.

uint8 sector_mask
snappy_interfaces/MotorStatus motors
snappy_interfaces/TempStatus temps
snappy_interfaces/AmbientStatus ambient
snappy_interfaces/PowerStatus power
snappy_interfaces/GroundFaultStatus ground_fault
snappy_interfaces/RailStatus rails
snappy_interfaces/DiagStatus diag
