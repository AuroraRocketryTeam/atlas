#pragma once

#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"
#include "IStateMachine.hpp"

#include <memory>
#include "airbrakes/PredictiveController.hpp"
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
                  std::shared_ptr<RocketLogger> logger,
                  IStateMachine* fsm);

    void onTaskStart() override;
    void onTaskStop() override;
    void taskFunction() override;

private:
    std::shared_ptr<RocketModel> _rocketModel;

    std::shared_ptr<RocketLogger> _logger;

    // Points at the single embedded Cd table
    airbrakes::ControllerParams _params{};

    IStateMachine* _fsm;

    /**
     * @brief Command the airbrakes deployment through the RocketModel.
     * @param deployment 0.0f fully closed, 1.0f fully open.
     */
    void airbrakesCommand(float deployment);

    // Open-close
    void airbrakes_ctl_law();
    void calibrate();
};
