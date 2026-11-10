#pragma once

#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"
#include "airbrakes/AeroTableCsv.hpp"

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

struct AirbrakesConfig {
    float opening_altitude_m;
    float target_apogee_m;
    float dry_mass_kg;
    float reference_area_m2;
    float prediction_dt_s;
    uint32_t prediction_max_steps;
};

class AirbrakesTask : public BaseTask
{
public:
    AirbrakesTask(std::shared_ptr<RocketModel> rocketModel,
                  std::shared_ptr<RocketLogger> logger);

    void onTaskStart() override;
    void onTaskStop() override;
    void taskFunction() override;

private:
    std::shared_ptr<RocketModel> _rocketModel;

    std::shared_ptr<RocketLogger> _logger;

    // Points at the single embedded Cd table
    airbrakes::ControllerParams _params{};
};
