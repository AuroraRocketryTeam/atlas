#pragma once

/**
 * @brief Interface for the actuator driving the airbrackes.
 */
class IAirbrakesController
{
public:
    virtual ~IAirbrakesController() = default;

    /**
     * @brief Init the actuator.
     * @return True if the actuator is ready.
     */
    virtual bool start() = 0;

    /**
     * @brief Stop the actuator and release its resources.
     */
    virtual void stop() = 0;

    /**
     * @brief Command the airbrakes deployment.
     * @param deployment 0.0f fully closed, 1.9f fully open.
     */
    virtual void command(float deployment) = 0;
};


