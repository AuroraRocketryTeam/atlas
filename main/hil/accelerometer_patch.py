def measure(self, time, **kwargs):
    """Measure acceleration at the sensor location in the sensor frame."""

    u = kwargs["u"]
    u_dot = kwargs["u_dot"]
    environment = kwargs["environment"]

    # Position of the sensor relative to the rocket CDM, in the body frame.
    r_body = Vector(kwargs["relative_position"])

    # Rocket CDM acceleration and gravity, both in the inertial frame.
    linear_acceleration_inertial = Vector(u_dot[3:6])
    gravity_magnitude = environment.gravity.get_value_opt(u[3])
    gravity_inertial = Vector([0, 0, -gravity_magnitude])

    # A physical accelerometer measures specific force:
    #
    #     f = a - g
    #
    # During ideal free fall, a = g and the accelerometer reads zero.
    if self.consider_gravity:
        linear_acceleration_inertial -= gravity_inertial

    # Transform CDM acceleration from the inertial frame to the body frame.
    inertial_to_body = Matrix.transformation(u[6:10]).transpose
    linear_acceleration_body = (
        inertial_to_body @ linear_acceleration_inertial
    )

    # Angular velocity and angular acceleration are expressed in the body frame.
    omega_body = Vector(u[10:13])
    omega_dot_body = Vector(u_dot[10:13])

    # Tangential acceleration caused by angular acceleration:
    #
    #     a_t = alpha × r
    tangential_acceleration_body = Vector.cross(
        omega_dot_body,
        r_body,
    )

    # Centripetal acceleration caused by angular velocity:
    #
    #     a_c = omega × (omega × r)
    centripetal_acceleration_body = Vector.cross(
        omega_body,
        Vector.cross(omega_body, r_body),
    )

    # All terms are now expressed in the same body frame.
    acceleration_at_sensor_body = (
        linear_acceleration_body
        + tangential_acceleration_body
        + centripetal_acceleration_body
    )

    # Transform the complete acceleration from body axes to clean sensor axes.
    sensor_to_body = self.rotation_sensor_to_body
    body_to_clean_sensor = sensor_to_body.transpose

    # RocketPy currently stores:
    #     _total_rotation_sensor_to_body = sensor_to_body @ cross_axis_matrix
    # Recover the output-space cross-axis matrix:
    clean_sensor_to_output = (
        sensor_to_body.transpose @ self._total_rotation_sensor_to_body
    )
    acceleration_sensor_clean = body_to_clean_sensor @ acceleration_at_sensor_body
    acceleration_sensor = clean_sensor_to_output @ acceleration_sensor_clean
    
    # Apply the configured sensor imperfections.
    acceleration_sensor = self.apply_noise(acceleration_sensor)
    acceleration_sensor = self.apply_temperature_drift(acceleration_sensor)
    acceleration_sensor = self.quantize(acceleration_sensor)

    self.measurement = tuple(acceleration_sensor)
    self._save_data((time, *acceleration_sensor))