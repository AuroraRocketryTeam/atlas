#pragma once

#include <cstdint>

/**
 * @file FlightFlags.hpp
 * @brief Has to survive a reset, so a board that comes back up
 *        in mid air can tell that is what happened.
 *
 * The launched flag is raised when the rocket is armed and lowered when it
 * reaches RECOVERED. A boot that finds it raised therefore happened between
 * those two moments. It is stored in NVS so that is persists.
 * 
 */
namespace FlightFlags
{
    /**
     * @brief Whether a flight is in progress
     * @return false when the flag is down, absent, or unreadable.
     */
    bool isLaunched();

    /**
     * @brief Set flag value
     *
     * @return true when the value reached NVS.
     */
    bool setLaunched(bool launched);

    /**
     * @brief How many times recovery has already been fired since arming.
     *
     * If deploying causes brownouts we use this to stop retrying.
     * It is cleared by setLaunched(true) at arming.
     */
    uint8_t deployAttempts();

    /** @brief Add one to the attempt count.  */
    bool recordDeployAttempt();
}
