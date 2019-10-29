---
layout: post
title: "OpenPilot Autonomous Driving Stack"
author: "Nick Rotella"
categories: journal
tags: [autonomous driving,control,open-source]
image: html.jpg
---

*A long while back, I played around with a [comma.ai Panda OBD-II Dongle](https://comma.ai/setup/panda) which is an inexpensive vehicle CAN bus interface designed to perform basic self-driving functionality using only an RGB camera and lightweight processor, both mounted behind the rear view mirror. My goal was to understand comma.ai's open-source "OpenPilot" codebase both as an intro to a basic self-driving control stack and to eventually (hopefully) use the Panda dongle either with OpenPilot or custom software to do some simple autonomous control, ideally with the inclusion of imaging radar. I didn't quite get that far due to shifting priorities, but I took some notes that I might as well post now that I'm looking back over them. I hope it helps someone somehow!* 

# OpenPilot

The [OpenPilot](https://github.com/commaai/openpilot) project is an open-source autonomous driving platform designed for comma.ai's products ([Panda OBD-II Dongle](https://comma.ai/setup/panda) and [EON](https://comma.ai/setup/eon) processor).  It contains python code for doing everything from interpreting CAN messages for different car types to performing model predictive control using a combination of sensors.  For a very simple but useful overview of the project, check out [this article](https://medium.com/@comma_ai/how-does-openpilot-work-c7076d4407b3) from an OpenPilot contributor.

Unfortunately, it is written specifically for the EON hardware/architecture and thus has dependencies which were not easily resolvable on an Ubuntu system (I also tried a Docker container but still couldn't use it).  While it may be possible to isolate certain functionality, we can also just use the project as a guide for implementing our own versions of specific modules.  For example, in our [Panda wiki](http://aiwiki.local/drive/panda) there is information on where to find files related to CAN parsing for our platform.

This page will serve to document some of the higher-level functionality (car modelling, control, sensor fusion etc) as needed.

## Package Structure

The OpenPilot package has the following structure.  At the top level are a number of different libraries on which the self-driving functionality, found in [openpilot/selfdrive](https://github.com/commaai/openpilot/tree/devel/selfdrive), depend on.  These include eg the [panda](https://github.com/commaai/openpilot/tree/devel/panda) module for CAN communication, [opendbc](https://github.com/commaai/openpilot/tree/devel/opendbc) for DBC parsing and so on.

### Controls

Within the selfdrive subpackage are different subsubpackages handling different aspects of self driving functionality.  Vehicle models and control algorithms which use these models are found in the [controls](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls) package, with [controlsd.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py) implementing high-level control using the rest of the [controls library]().

The [main control loop](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py#L399) is set to run in a thread at 100Hz.  Let's take a closer look at what it's doing:

```python
def controlsd_thread(gctx=None, rate=100, default_bias=0.):
  gc.disable()

  # start the loop
  set_realtime_priority(3)

  context = zmq.Context()
  params = Params()

  # Pub Sockets
  live100 = messaging.pub_sock(context, service_list['live100'].port)
  carstate = messaging.pub_sock(context, service_list['carState'].port)
  carcontrol = messaging.pub_sock(context, service_list['carControl'].port)
  livempc = messaging.pub_sock(context, service_list['liveMpc'].port)

  is_metric = params.get("IsMetric") == "1"
  passive = params.get("Passive") != "0"

  # No sendcan if passive
  if not passive:
    sendcan = messaging.pub_sock(context, service_list['sendcan'].port)
  else:
    sendcan = None

  # Sub sockets
  poller = zmq.Poller()
  thermal = messaging.sub_sock(context, service_list['thermal'].port, conflate=True, poller=poller)
  health = messaging.sub_sock(context, service_list['health'].port, conflate=True, poller=poller)
  cal = messaging.sub_sock(context, service_list['liveCalibration'].port, conflate=True, poller=poller)
  driver_monitor = messaging.sub_sock(context, service_list['driverMonitoring'].port, conflate=True, poller=poller)
  gps_location = messaging.sub_sock(context, service_list['gpsLocationExternal'].port, conflate=True, poller=poller)
  logcan = messaging.sub_sock(context, service_list['can'].port)
```

First, it creates communication sockets using [messaging.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/messaging.py) which is a simple wrapper for [ZeroMQ](http://zeromq.org/) distributed messaging using [Cap'n Proto](https://capnproto.org/) for serialization of python objects (see eg [car.capnp](https://github.com/commaai/openpilot/blob/devel/cereal/car.capnp) to understand how serialization is implemented for car-related data structures).  

```python
  CC = car.CarControl.new_message()
  CI, CP = get_car(logcan, sendcan, 1.0 if passive else None)

  if CI is None:
    raise Exception("unsupported car")

  # if stock camera is connected, then force passive behavior
  if not CP.enableCamera:
    passive = True
    sendcan = None

  if passive:
    CP.safetyModel = car.CarParams.SafetyModels.noOutput

  # Get FCW toggle from settings
  fcw_enabled = params.get("IsFcwEnabled") == "1"
  geofence = None
```
	
The serialization of a [CarControl](https://github.com/commaai/openpilot/blob/devel/cereal/car.capnp#L210) capnp structure is used to send car commands which are computed in the remainder of the thread loop.  The [car](https://github.com/commaai/openpilot/tree/devel/selfdrive/car) package is also used here to get a car information object through a generic interface defined by [carhelpers.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/car_helpers.py).  This return a car interface and car parameters objects which store physical parameters, settings etc.

```python
  PL = Planner(CP, fcw_enabled)
  LoC = LongControl(CP, CI.compute_gb)
  VM = VehicleModel(CP)
  LaC = LatControl(CP)
  AM = AlertManager()
  driver_status = DriverStatus()

  if not passive:
    AM.add("startup", False)

  # Write CarParams for radard and boardd safety mode
  params.put("CarParams", CP.to_bytes())
  params.put("LongitudinalControl", "1" if CP.openpilotLongitudinalControl else "0")
```

The car interface and parameter objects are then used to create and initialize car planners and controllers.

```python
  state = State.disabled
  soft_disable_timer = 0
  v_cruise_kph = 255
  v_cruise_kph_last = 0
  overtemp = False
  free_space = False
  cal_status = Calibration.INVALID
  cal_perc = 0
  mismatch_counter = 0
  low_battery = False

  rk = Ratekeeper(rate, print_delay_threshold=2. / 1000)

# Read angle offset from previous drive, fallback to default
  angle_offset = default_bias
  calibration_params = params.get("CalibrationParams")
  if calibration_params:
    try:
      calibration_params = json.loads(calibration_params)
      angle_offset = calibration_params["angle_offset2"]
    except (ValueError, KeyError):
      pass

  prof = Profiler(False)  # off by default
```
	
Here the initial state machine state is initialized along with other parameters before entering the main loop.  The ratekeeper is defined in [realtime.py](https://github.com/commaai/openpilot/blob/devel/common/realtime.py) along with other utilities for maintaining realtime loops and setting thread priority.  The profile is a utility for printing times of checkpoints, defined in [profiler.py](https://github.com/commaai/openpilot/blob/devel/common/profiler.py).

```python
  while True:
    prof.checkpoint("Ratekeeper", ignore=True)

    # Sample data and compute car events
    CS, events, cal_status, cal_perc, overtemp, free_space, low_battery, mismatch_counter = data_sample(CI, CC, thermal, cal, health,
      driver_monitor, gps_location, poller, cal_status, cal_perc, overtemp, free_space, low_battery, driver_status, geofence, state, mismatch_counter, params)
    prof.checkpoint("Sample")

    # Define longitudinal plan (MPC)
    plan, plan_ts = calc_plan(CS, CP, VM, events, PL, LaC, LoC, v_cruise_kph, driver_status, geofence)
    prof.checkpoint("Plan")
```

The main loop for the control thread first gets updated car data using [data_sample](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py#L42) which polls for sensor data and sets events (eg low battery, high temperature, etc) and then computes a plan using model predictive control (MPC) in [calc_plan](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py#L122).  This function uses the Planner class of [planner.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/planner.py), which gets updated by passing in longitudinal and lateral planners to [update](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/planner.py#L341).  As noted, only the longitudinal MPC is [defined](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/planner.py#L141) in the planner; the lateral MPC is defined in the [lateral controller](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/latcontrol.py).

For both lat/long MPCs, the actual optimization uses [cffi](https://cffi.readthedocs.io/en/latest/) to interface python with the C++ [Acado](http://acado.github.io/) optimization toolkit which uses [qpOASES](https://github.com/commaai/openpilot/tree/devel/phonelibs/qpoases) as a QP solver.  The MPC problems are defined in C in [longitudinal_mpc.c](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/longitudinal_mpc/longitudinal_mpc.c) and [lateral_mpc.c](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/lateral_mpc/lateral_mpc.c); these are compiled against the optimization libraries and given python interfaces using cffi in [longitudinal_mpc/libmpc_py.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/longitudinal_mpc/libmpc_py.py) and [lateral_mpc/libmpc_py.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/lateral_mpc/libmpc_py.py) respectively.

The libmpc_py.py interface files for each planner provide the get_mpc function which returns an interface to the MPC variables and the init and run_mpc functions which are ultimately used by the python planners to solve the MPC problems.

```python
    if not passive:
      # update control state
      state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last = \
        state_transition(CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM)
      prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, v_cruise_kph, driver_status, angle_offset = state_control(plan, CS, CP, state, events, v_cruise_kph,
      v_cruise_kph_last, AM, rk, driver_status, PL, LaC, LoC, VM, angle_offset, passive, is_metric, cal_perc)
    prof.checkpoint("State Control")

    # Publish data
    CC = data_send(PL.perception_state, plan, plan_ts, CS, CI, CP, VM, state, events, actuators, v_cruise_kph, rk, carstate, carcontrol,
                   live100, livempc, AM, driver_status, LaC, LoC, angle_offset, passive)
    prof.checkpoint("Sent")

    rk.keep_time()  # Run at 100Hz
    prof.display()
```

In function [state_control](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py#L272) the [lateral control](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/latcontrol.py) and [longitudinal control](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/lib/longcontrol.py) get updated.  Since the longitudinal MPC was computed above, only its feedback look is updated here; the lateral control has both MPC and feedback loop updated at this point.  The output of the lat/lon controllers are actuator commands.  

Before sending actuator commands, the state machine state is updated in [state_transition](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py#L141) by checking for events triggered previously (in the data_sample or calc_plan functions). 
## Sending Control Commands

Parsing CAN data is straightforward using the correct DBC file, but sending commands to eg set the steering angle is more involved.  The point in the controller at which actuator commands are sent is [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/controls/controlsd.py#L327), using a car interface object and a capnp CarControl message loaded with computed commands. 

### Car Interfaces

The car interface is returned from [get_car](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/car_helpers.py#L87) which uses some nifty functionality defined in [common/fingerprints.py](https://github.com/commaai/openpilot/blob/devel/common/fingerprints.py) to automatically return the correct car interface by matching current CAN data with candidates in the library of known vehicles.

Interfaces are different for different manufacturers; the library of interfaces is loaded [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/car_helpers.py#L39) by iterating through the [car](https://github.com/commaai/openpilot/blob/devel/selfdrive/car) package and importing *interface.py* for each manufacturer (eg for [honda](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/interface.py)).

Every car interface has an [apply](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/interface.py#L594) function which takes as input a CarControl message structure containing commands.  Every interface imports a [CarController](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/carcontroller.py) class specific to that manufacturer which implements parsing the CarControl message into interface-specific commands inside its [update](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/carcontroller.py#L87) function.

Within the ```update```, functions like ```create_steering_control```, ```create_gas_command``` etc are called from the manufacturer-specific CAN interface, eg [hondacan.py](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/hondacan.py) using a [CANPacker](https://github.com/commaai/openpilot/blob/devel/selfdrive/can/packer.py) . These CAN messages get appended to a list of messages and are finally sent [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/carcontroller.py#L184).

Note that ```create_steering_control``` passes along ```lkas_active```, which is related to the Lane Keep Assist System which comes stock with the car; without LKAS enabled for the car, OpenPilot cannot control the steering.  The variable ```lkas_active``` is set to ```enabled``` [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/carcontroller.py#L148) unless steering has been disabled from the CarState; this is determined [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/carstate.py#L206).

### Sending our own controls

Let's look at what it would take to send a steering command.  We actually send steer torques, so to command a steering angle we'll need to implement a feedback loop for torque. The steering command to apply is computed [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/car/honda/carcontroller.py#L146) in the car controller, clipped to min/max hex values:

```python
    # *** compute control surfaces ***
    BRAKE_MAX = 1024/4
    if CS.CP.carFingerprint in (CAR.ACURA_ILX):
      STEER_MAX = 0xF00
    elif CS.CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX):
      STEER_MAX = 0x3e8  # CR-V only uses 12-bits and requires a lower value (max value from energee)
    else:
      STEER_MAX = 0x1000

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = int(clip(self.brake_last * BRAKE_MAX, 0, BRAKE_MAX - 1))
apply_steer = int(clip(-actuators.steer * STEER_MAX, -STEER_MAX, STEER_MAX))
```

where the limits of the actuator apply_steer value are found in the [capnp message definition](https://github.com/commaai/openpilot/blob/devel/cereal/car.capnp#L224):

```
  struct Actuators {
    # range from 0.0 - 1.0
    gas @0: Float32;
    brake @1: Float32;
    # range from -1.0 - 1.0
    steer @2: Float32;
    steerAngle @3: Float32;
}
```

In the case of our Acura ILX, the max is 0xF00 which is 3840.  This corresponds to the ```STEERING_CONTROL``` [DBC message format](https://github.com/commaai/openpilot/blob/devel/opendbc/acura_ilx_2016_can_generated.dbc#L236) which includes min/max:

```
BO_ 228 STEERING_CONTROL: 5 ADAS
 SG_ STEER_TORQUE : 7|16@0- (1,0) [-3840|3840] "" EPS
 SG_ STEER_TORQUE_REQUEST : 23|1@0+ (1,0) [0|1] "" EPS
 SG_ SET_ME_X00 : 31|8@0+ (1,0) [0|0] "" EPS
 SG_ COUNTER : 37|2@0+ (1,0) [0|3] "" EPS
SG_ CHECKSUM : 35|4@0+ (1,0) [0|3] "" EPS
```

## Compiling OpenDBC outside OpenPilot

While OpenPilot has a lot of great functionality, we really just want to pull out the modules which read DBC files, interpret incoming CAN data and allow sending commands - all while being agnostic to the vehicle make/model.  

The bulk of this functionality is in the [can](https://github.com/commaai/openpilot/tree/devel/selfdrive/can) module, which relies on the [cereal](https://github.com/commaai/openpilot/tree/devel/cereal) module for serialization.  The can module also uses the [opendbc](https://github.com/commaai/openpilot/tree/devel/opendbc) module which contains all generated DBC files to be loaded.  There is also dependency on the [common](https://github.com/commaai/openpilot/tree/devel/common) module.  These should be able to be isolated and built separately.  I pulled out the can module and put the opendbc and cereal modules inside it, and pulled out the common module and placed it beside can.

Building these requires modifying Makefiles to replace phone libs with system libs (including capnproto and zmq). I modified [this line](https://github.com/commaai/openpilot/blob/devel/selfdrive/can/Makefile#L17) in the can Makefile to use c++14:

```
CFLAGS = -std=gnu14 -g -fPIC -O2 $(WARN_FLAGS)
CXXFLAGS = -std=c++14 -g -fPIC -O2 $(WARN_FLAGS)
```

And also modified [these lines](https://github.com/commaai/openpilot/blob/devel/selfdrive/can/Makefile#L24) to use the system install of zmq:

```
else ifeq ($(UNAME_M),x86_64)
        ZMQ_FLAGS = -I/usr/local/include/zmq
        ZMQ_LIBS = -L/usr/local/lib -l:libzmq.a
```

 Compilation requires Clang and Clang++ so I installed ```sudo apt install clang-3.8``` and created symlinks with ```sudo ln -s /usr/bin.clang-3.8 /usr/bin/clang``` and ```sudo ln -s /usr/bin.clang++-3.8 /usr/bin/clang++```.

The cereal Makefile got modifed [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/common/cereal.mk#L6) to specify the system install of capnp:

```
CEREAL_CFLAGS = -I$/usr/local/include/capnp/
```

And same [here](https://github.com/commaai/openpilot/blob/devel/selfdrive/common/cereal.mk#L19):

```
else ifeq ($(UNAME_M),x86_64)

CEREAL_CXXFLAGS = -I/usr/local/include/capnp
ifeq ($(CEREAL_LIBS),)
  CEREAL_LIBS = -L/usr/local/lib/ \
                -L/usr/local/lib/ \
                -l:libcapnp.a -l:libkj.a -l:libcapnp_c.a
endif
```

The end result when trying to compile can (which tries to compile cereal as a prereq) was tons of c++14 related errors, for some reason. 
