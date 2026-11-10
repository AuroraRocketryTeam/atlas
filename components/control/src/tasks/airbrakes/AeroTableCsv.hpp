#pragma once

#include "PredictiveController.hpp"

#include <cstddef>

/**
 * @brief The airbrake Cd table, loaded from `airbrakes_cd_table.csv`.
 *
 * The table is aero data, not an operator setting, so it lives in a CSV file
 * next to the controller instead of in RuntimeConfig. The file is embedded in
 * the firmware image at build time and parsed once; the parsed table has a
 * single static instance that the controller references, so it costs no
 * RuntimeConfig, NVS or task-stack space.
 */
namespace airbrakes
{
    /** Result of loading the CSV. Messages are plain ASCII without quotes, safe to embed in JSON. */
    struct AeroTableLoad
    {
        /** True when the table parsed and passed every check; the controller refuses to run otherwise. */
        bool ok;
        /** False when Cd decreases with opening somewhere (allowed, but the controller assumes more opening means more drag). */
        bool cd_monotonic;
        /** Why loading failed, with the CSV line number. Empty when ok. */
        char error[96];
        AeroTable table;
    };

    /**
     * @brief Parse and validate the CSV text of a Cd table.
     *
     * Format: '#' comments and blank lines are skipped. The first data row is a
     * label cell followed by the Mach breakpoints; each following row is an
     * opening in percent followed by one Cd per Mach breakpoint.
     *
     * @param text CSV text; parsing stops at @p length bytes or a NUL.
     * @param length Number of bytes available at @p text.
     * @param out Receives the table and the verdict.
     * @return out.ok
     */
    bool parseAeroTableCsv(const char *text, size_t length, AeroTableLoad &out);

    /**
     * @brief The table embedded from airbrakes_cd_table.csv, parsed on first use.
     *
     * Defined in EmbeddedAeroTable.cpp (firmware only).
     */
    const AeroTableLoad &embeddedAeroTable();
}
