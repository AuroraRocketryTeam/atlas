This version is still using the modified version of the RocketPy simulator with the ```RocketV2``` class that exposes ```add_state_and_sensors_logger()``` method.
Consider changing it to the wrapper in order to not depend on the internal code structure of RocketPy.

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

### 3. Modify RocketPy
- add ```rocket_v2.py``` in ```RocketPy/rocketpy/rocket/```,
- modify the ```__init__.py``` in ```RocketPy/rocketpy``` and expose ```RocketV2``` class:
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
- modify the ```__init__.py``` in ```RocketPy/rocketpy/rocket``` and expose ```RocketV2``` class:
```python
from rocketpy.rocket.rocket_v2 import RocketV2   # modify this line
```
- <!-- - change ```RocketPy/rocketpy/simulation/flight.py#L3746```, comment ```tmp_dict[time]._controllers += node._controllers``` (duplicated, adding twice the controllers). -->

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
  python3 hil_rocketpy.py
  ```