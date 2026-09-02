#pragma once

#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Airbrake tuning belongs to this task; RuntimeConfig only composes it for
// persistence and inspection.  It is not a generic system-wide bag of fields.
struct AirbrakesConfig {
    float open_altitude_m;
    float close_altitude_m;
    float open_rate_per_s;
    float close_rate_per_s;
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
};
