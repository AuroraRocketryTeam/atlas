#include "AeroTableCsv.hpp"

extern const char airbrakes_cd_table_csv_start[] asm("_binary_airbrakes_cd_table_csv_start");
extern const char airbrakes_cd_table_csv_end[] asm("_binary_airbrakes_cd_table_csv_end");

const AeroTableLoad &embeddedAeroTable()
{
    static const AeroTableLoad table = [] {
        AeroTableLoad load;
        parseAeroTableCsv(airbrakes_cd_table_csv_start,
                          static_cast<size_t>(airbrakes_cd_table_csv_end - airbrakes_cd_table_csv_start),
                          load);
        return load;
    }();
    return table;
}
