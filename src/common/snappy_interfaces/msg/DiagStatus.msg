# Board health. Topic: /mower/diag

uint8 RESET_LOW_POWER=1
uint8 RESET_WWDG=2
uint8 RESET_IWDG=4            # watchdog reset: firmware hung
uint8 RESET_SOFTWARE=8        # includes fault and stack-overflow resets
uint8 RESET_POWER_ON=16
uint8 RESET_PIN=32
uint8 RESET_BROWNOUT=64

builtin_interfaces/Time stamp
uint32 uptime_s
uint8 reset_cause             # RESET_* bits of the last reset
uint32 heap_free              # FreeRTOS heap bytes
uint32 heap_min
uint32 uros_heap_free         # micro-ROS heap bytes
uint32 uros_heap_min
uint32 uros_alloc_failures
uint32 uros_stack_min         # bytes never used, micro-ROS task
uint32 sensor_stack_min       # bytes never used, sensor task
uint32 i2c1_errors
uint32 i2c2_errors
uint32 i2c_recoveries
uint32 uart_rx_restarts       # serial RX restarts after line errors
uint32 agent_connects
uint32 ping_failures
float32 mcu_temp_c
float32 vdda_v
bool watchdog_active
bool time_synced              # stamps use agent (ROS) time
