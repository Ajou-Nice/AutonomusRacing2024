float32 gps_tow
uint16  gps_week_number
uint8   sbas_system  # Must be equal to one of the SBAS_SYSTEM_* values in this message
uint8   sbas_id
uint8   count

# Parsed out version of the SBAS status bitfield
bool status_range_available
bool status_corrections_available
bool status_integrity_available
bool status_test_mode

# Valid flags can be parsed per the manual to determine which fields are valid
uint16 valid_flags

# Possible values for the sbas_system field
uint8 SBAS_SYSTEM_UNKNOWN = 0
uint8 SBAS_SYSTEM_WAAS = 1
uint8 SBAS_SYSTEM_EGNOS = 2
uint8 SBAS_SYSTEM_MSAS = 3
uint8 SBAS_SYSTEM_GAGAN = 4