# Isolation (ground fault) monitor on GFout. Topic: /mower/ground_fault

int8 SIDE_NONE=0
int8 SIDE_NEGATIVE=-1         # battery negative leaking to sea
int8 SIDE_POSITIVE=1          # battery positive (VBAT) leaking to sea

builtin_interfaces/Time stamp
uint16 adc_raw                # averaged 12-bit ADC counts
float32 gfout_v               # GFout pin voltage
float32 bridge_v              # bridge node B vs SEA, zero corrected
float32 baseline_v            # GFout treated as no leak
float32 leak_kohm             # estimated leak; 100000 = none measurable
int8 side                     # SIDE_*
bool fault                    # leak_kohm < trip_kohm
float32 trip_kohm
float32 vbat_v                # battery voltage assumed by the estimate
bool valid
