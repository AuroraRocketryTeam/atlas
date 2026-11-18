#include "AeroTableCsv.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

constexpr size_t kMaxMach = AirbrakesController::kMaxMachBreakpoints;
constexpr size_t kMaxOpening = AirbrakesController::kMaxOpeningBreakpoints;
// Widest row: an opening plus one Cd per Mach breakpoint (header: label plus Mach breakpoints)
constexpr size_t kMaxCells = (kMaxMach > kMaxOpening ? kMaxMach : kMaxOpening) + 1;
constexpr size_t kMaxCellLength = 31; // Longest numeric cell accepted

static bool fail(AeroTableLoad &out, unsigned line, const char *message)
{
    out.ok = false;
    if (line > 0) std::snprintf(out.error, sizeof(out.error), "airbrakes_cd_table.csv line %u: %s", line, message);
    else std::snprintf(out.error, sizeof(out.error), "airbrakes_cd_table.csv: %s", message);
    return false;
}

/** Parse one trimmed cell as a finite float, rejecting trailing garbage. */
static bool parseNumber(const char *begin, const char *end, float &value)
{
    while (begin < end && std::isspace(static_cast<unsigned char>(*begin))) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(end[-1]))) --end;
    const size_t length = static_cast<size_t>(end - begin);
    if (length == 0 || length > kMaxCellLength) return false;

    char cell[kMaxCellLength + 1];
    std::memcpy(cell, begin, length);
    cell[length] = '\0';
    char *parsedEnd = nullptr;
    value = std::strtof(cell, &parsedEnd);
    return parsedEnd == cell + length && std::isfinite(value);
}

static bool strictlyIncreasing(const float *values, size_t count)
{
    for (size_t i = 1; i < count; ++i)
    {
        if (!(values[i] > values[i - 1])) return false;
    }
    return true;
}

bool parseAeroTableCsv(const char *text, size_t length, AeroTableLoad &out)
{
    out = AeroTableLoad{};
    out.cd_monotonic = true;
    if (text == nullptr) return fail(out, 0, "file is missing");

    AirbrakesController::AeroTable &table = out.table;
    bool headerSeen = false;
    unsigned lineNumber = 0;
    size_t position = 0;

    while (position < length && text[position] != '\0')
    {
        // Isolate one line, without its CR/LF.
        const char *lineBegin = text + position;
        size_t lineLength = 0;
        while (position + lineLength < length && lineBegin[lineLength] != '\0' && lineBegin[lineLength] != '\n')
        {
            ++lineLength;
        }
        position += lineLength;
        if (position < length && text[position] == '\n') ++position;
        ++lineNumber;

        const char *lineEnd = lineBegin + lineLength;
        if (lineEnd > lineBegin && lineEnd[-1] == '\r') --lineEnd;
        const char *first = lineBegin;
        while (first < lineEnd && std::isspace(static_cast<unsigned char>(*first))) ++first;
        if (first == lineEnd || *first == '#') continue;

        // Split on commas.
        const char *cellBegin[kMaxCells];
        const char *cellEnd[kMaxCells];
        size_t cellCount = 0;
        const char *cursor = first;
        while (true)
        {
            if (cellCount == kMaxCells) return fail(out, lineNumber, "more than 8 Mach columns");
            const char *comma = static_cast<const char *>(std::memchr(cursor, ',', static_cast<size_t>(lineEnd - cursor)));
            cellBegin[cellCount] = cursor;
            cellEnd[cellCount] = comma != nullptr ? comma : lineEnd;
            ++cellCount;
            if (comma == nullptr) break;
            cursor = comma + 1;
        }

        if (!headerSeen)
        {
            // Header: label cell, then the Mach breakpoints.
            headerSeen = true;
            if (cellCount < 3) return fail(out, lineNumber, "header needs a label and at least 2 Mach breakpoints");
            for (size_t c = 1; c < cellCount; ++c)
            {
                if (!parseNumber(cellBegin[c], cellEnd[c], table.mach_bp[c - 1]))
                    return fail(out, lineNumber, "Mach breakpoint is not a number");
            }
            table.mach_count = static_cast<uint8_t>(cellCount - 1);
            if (!strictlyIncreasing(table.mach_bp, table.mach_count) || table.mach_bp[0] < 0.0f)
                return fail(out, lineNumber, "Mach breakpoints must be increasing and at least 0");
            continue;
        }

        // Data row: opening, then one Cd per Mach breakpoint.
        if (table.opening_count == kMaxOpening) return fail(out, lineNumber, "more than 8 opening rows");
        if (cellCount != static_cast<size_t>(table.mach_count) + 1)
            return fail(out, lineNumber, "row must have an opening and one Cd per Mach breakpoint");

        const size_t row = table.opening_count;
        if (!parseNumber(cellBegin[0], cellEnd[0], table.opening_bp_pct[row]))
            return fail(out, lineNumber, "opening is not a number");
        for (size_t c = 1; c < cellCount; ++c)
        {
            float &cd = table.cd[row][c - 1];
            if (!parseNumber(cellBegin[c], cellEnd[c], cd)) return fail(out, lineNumber, "Cd is not a number");
            if (cd <= 0.0f || cd > 5.0f) return fail(out, lineNumber, "Cd must be greater than 0 and at most 5");
            if (row > 0 && cd < table.cd[row - 1][c - 1]) out.cd_monotonic = false;
        }
        table.opening_count = static_cast<uint8_t>(row + 1);
    }

    if (!headerSeen) return fail(out, 0, "no header row with Mach breakpoints");
    if (table.opening_count < 2) return fail(out, 0, "needs at least 2 opening rows");
    if (!strictlyIncreasing(table.opening_bp_pct, table.opening_count) ||
        table.opening_bp_pct[0] < 0.0f || table.opening_bp_pct[table.opening_count - 1] > 100.0f)
    {
        return fail(out, 0, "openings must be increasing and within 0-100 percent");
    }

    out.ok = true;
    return true;
}
