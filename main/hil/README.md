

### 1. Clone RocketPy
```bash
git clone --depth=1 https://github.com/RocketPy-Team/RocketPy.git
cd RocketPy
```

### 2. Python Environment
```bash
python3 -m venv venv_dev_rocketpy
source venv_dev_rocketpy/bin/activate
```

> [!NOTE] This version is still using the modified version of the RocketPy simulator with the RocketV2 class that exposes "add_state_and_sensors_logger()" method. Consider changing it to the wrapper in order to not depend on the internal code structure of RocketPy.
### 3. Modify RocketPy
1. add ```rocket_v2.py``` in ```RocketPy/rocketpy/rocket/```,
2. modify the ```__init__.py``` in ```RocketPy/rocketpy``` and expose ```RocketV2``` class:
```python
from .rocket import (
    AeroSurface,
    AirBrakes,
    Components,
    EllipticalFins,
    Fins,
    FreeFormFins,
    GenericSurface,
    LinearGenericSurface,
    NoseCone,
    Parachute,
    PointMassRocket,
    RailButtons,
    RocketV2, # modify this line
    Tail,
    TrapezoidalFins)
```
3. modify the ```__init__.py``` in ```RocketPy/rocketpy/rocket``` and expose ```RocketV2``` class:
```python
from rocketpy.rocket.rocket_v2 import RocketV2   # modify this line
```

Bugs:
1. [THIS HAS BEEN FIXED] ~~~change ```RocketPy/rocketpy/simulation/flight.py#L3746```, comment ```tmp_dict[time]._controllers += node._controllers``` (duplicated, adding twice the controllers).~~~
2. Controllers callback are called twice: 
   - [self.__process_sensors_and_controllers_at_current_node(node, phase)](https://github.com/RocketPy-Team/RocketPy/blob/cb15a393ee2d9430cc21c57c98768dc1890a198a/rocketpy/simulation/flight.py#L699-L700)
   - COMMENT THIS ONE OUT in `flight.py#L701`: [for controller in node._controllers:](https://github.com/RocketPy-Team/RocketPy/blob/cb15a393ee2d9430cc21c57c98768dc1890a198a/rocketpy/simulation/flight.py#L701)
3. `accelerometer.py` measure method gives weird values. Instead of -1g+1g=0g, it was giving -1g-1g=-2g and also there was a mix in the reference frames.
   - Modify the Accelerometer class in `rocketpy/sensors/accelerometer.py`. Patch the `measure()` method with the code present in the `accelerometer_patch.py`.
   - Modify the Sensor class in `rocketpy/sensors/sensor.py`. Make the `cross_axis_matrix` an attribute in order to make it visible outside the `init()` method:
      ```python
      self.cross_axis_matrix =  [...] # add self.

      self._total_rotation_sensor_to_body = (
          self.rotation_sensor_to_body @ self.cross_axis_matrix # add self.
      )
      ```

### 4. Install RocketPy from source
```bash
pip install -r requirements.txt
pip install .
```

### 5. Run the simulation
- build ```main_hil.cpp``` and flash ```atlas-esp-idf```.
- connect to the WiFi AP, ssid=```myssid``` and psw=```mypassword```.
- ```bash
  cd main/hil
  python3 hil_rocketpy.py --rocket=fred --sensor-profile=clean
  python3 hil_rocketpy.py --rocket=nemesis --sensor-profile=clean
  ```