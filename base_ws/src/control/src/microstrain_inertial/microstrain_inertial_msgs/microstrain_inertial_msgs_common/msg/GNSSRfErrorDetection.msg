uint8 rf_band  # Must be one of the RF_BAND_* values
uint8 jamming_state  # Must be one of JAMMING_STATE_* values
uint8 spoofing_state  # Must be one of SPOOFING_STATE_* values
uint16 valid_flags

# Enum values for rf_band field
uint8 RF_BAND_UNKNOWN = 0
uint8 RF_BAND_L1 = 1
uint8 RF_BAND_L2 = 2
uint8 RF_BAND_L5 = 5

# Enum values for jamming_state field
uint8 JAMMING_STATE_UNKNOWN = 0
uint8 JAMMING_STATE_NONE = 1
uint8 JAMMING_STATE_PARTIAL = 2
uint8 JAMMING_STATE_SIGNIFICANT = 3

# Enum values for spoofing_state field
uint8 SPOOFING_STATE_UNKNOWN = 0
uint8 SPOOFING_STATE_NONE = 1
uint8 SPOOFING_STATE_PARTIAL = 2
uint8 SPOOFING_STATE_SIGNIFICANT = 3