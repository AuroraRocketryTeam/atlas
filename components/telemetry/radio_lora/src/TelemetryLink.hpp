#pragma once

#include "sdkconfig.h"

/**
 * @file TelemetryLink.hpp
 * @brief Selects where the telemetry goes, throw error if conflict settings.
 * 
 * Maybe in the future we can support both.
 */

#if defined(CONFIG_TELEMETRY_LINK_E220) && defined(CONFIG_TELEMETRY_LINK_ANTENNA_BOARD)
#error "Telemetry link: both the E220 module and the antenna board are enabled."
#endif

#include <SerialLogger.hpp>
#include <memory>

#if CONFIG_TELEMETRY_LINK_ANTENNA_BOARD
#include "AntennaBoardTransmitter.hpp"
using TelemetryLink = AntennaBoardTransmitter;
#else
#include "E220LoRaTransmitter.hpp"
#include <board.h>
using TelemetryLink = E220LoRaTransmitter;
#endif

/**
 * @brief Build the link this firmware was configured for and bring it up.
 * @return The link, or nullptr when its hardware did not answer.
 */
inline std::shared_ptr<TelemetryLink> createTelemetryLink()
{
#if CONFIG_TELEMETRY_LINK_ANTENNA_BOARD
    auto link = std::make_shared<AntennaBoardTransmitter>();
    const auto status = link->init();
#else
    // Serial1 is used by GPS, so the LoRa module gets Serial2.
    auto link = std::make_shared<E220LoRaTransmitter>(Serial2, MANNY_LORA_TX_PIN, MANNY_LORA_RX_PIN,
                                                      MANNY_LORA_AUX_PIN, MANNY_LORA_M0_PIN,
                                                      MANNY_LORA_M1_PIN);
    const auto status = link->init(E220LoRaTransmitter::defaultConfiguration());
#endif

    if (!status.isOk())
    {
        LOG_ERROR("Telemetry", "Link unavailable: %s", status.getDescription().c_str());
        return nullptr;
    }

    LOG_INFO("Telemetry", "Link ready: %s", status.getDescription().c_str());
    return link;
}
