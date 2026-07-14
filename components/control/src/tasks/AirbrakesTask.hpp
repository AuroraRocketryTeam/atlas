#pragma once

#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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