# Motor-area temperatures, 4x TMP1075. Topic: /mower/temps
# sensor_id: 11 = I2C1 0x48, 12 = I2C1 0x49, 21 = I2C2 0x48, 22 = I2C2 0x49

builtin_interfaces/Time stamp
uint8[4] sensor_id
float32[4] temp_c
bool[4] valid
bool[4] over_limit            # temp >= limit_c, clears 5 C below
float32 limit_c               # alert limit, set with /mower/config
bool smba1_alert              # I2C1 ALERT line active (sensor 11 or 12)
bool smba2_alert              # I2C2 ALERT line active (sensor 21 or 22)
