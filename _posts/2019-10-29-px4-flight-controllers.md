---
layout: post
title: "PX4 Flight Controllers"
author: "Nick Rotella"
categories: journal
tags: [drones,control,open-source]
image: px4.png
---

# PX4 Controllers and Tuning
The PX4 documentation does a decent job of explaining the different [flight controllers](https://dev.px4.io/en/flight_stack/controller_diagrams.html) which are available.  The documentation also has [useful information on how to set and get flight control parameters](https://dev.px4.io/en/advanced/parameters_and_configurations.html), which is necessary for controller tuning (among other things).  The documentation doesn't take the next step and explain how to tune the controllers, which is the purpose of this page.

In general, PX4 modules are documented using Doxygen which can either be used to build documentation or view it online [here](https://px4.github.io/Firmware-Doxygen/).  It's a lot to sift through, so we'll focus on snippets of code pasted to this page for now. 

For a good summary of the PX4 framework from an academic perspective, see this [ICRA paper](https://drive.google.com/open?id=13G7t0z0V-mnkLzosTRaSxryE14codVNM) written by the developers.

Contents:

1. [Multicopter Position and Velocity Controller](#multicopter-position-and-velocity-controller)
    a. [A Note on Vehicle Constraints](#a-note-on-vehicle-constraints)
2. [Multicopter Attitude Controller](#multicopter-attitude-controller)

## Multicopter Position and Velocity Controller
The multicopter position control diagram is shown below.

![px4_position_controller.png](../assets/img/px4_position_controller.png "PX4 MultiCopter Position Controller Diagram"){: .center-image}

The setpoint position $$r_{sp}$$ is specified from an offboard controller via the MAVROS topic [mavros/setpoint_position/local](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.setpoint_position), which is actually a full 6d pose ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) but only the position portion is used.  In actuality, this controller is a position *and velocity* controller; it will use a velocity setpoint $$v_{sp}$$ "from another module" specified via the topic [mavros/setpoint_velocity/cmd_vel_unstamped](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.setpoint_velocity) which again is 6d ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) but only the linear portion is used.  Although not explicitly shown in the diagram, the desired velocity $$v_{des}$$ output from the "mode" block is a combination of feedfoward velocity $$v_{sp}$$ and a feedback velocity computed from $$r_{sp}$$:

$$
v_{des} = v_{sp} + P(r_{sp}-\hat{r})
$$

where the proportional gains in forward ($$x$$) and lateral ($$y$$) directions, ```MPC_XY_P```, are the same and the proportional gain for altitude ($$z$$) control is ```MPC_Z_P```.  Note however that PID control is performed in the local NED (North-East-**Down**) frame, so a positive velocity in $$z$$ means the drone is losing altitude.  The velocity reference is passed to a cascaded (in other words, in series) velocity controller.  This is simply a PID controller which directly outputs a desired force $$F_{sp}$$:

$$
F_{sp} = PID(\hat{v}, v_{des})
$$

again with identical PID gains for $$x$$ and $$y$$ directions -  ```MPC_XY_VEL_P, MPC_XY_VEL_D, MPC_XY_VEL_I``` and $$z$$ direction gains ```MPC_Z_VEL_P, MPC_Z_VEL_D, MPC_Z_VEL_I```.  Of course, arbitrary forces cannot be applied to the drone; only the force along the instantaneous body z-axis can be realized.  The desired force is thus projected onto this axis to compute a desired thrust $$Z_{sp}$$ which is sent to the low-level mixing controller to be realized. 

The whole controller is implemented in the class [MulticopterPositionControl](https://px4.github.io/Firmware-Doxygen/dd/d4e/class_multicopter_position_control.html) in [src/modules/mc_pos_control/mc_pos_control_main.cpp](https://px4.github.io/Firmware-Doxygen/dd/d8c/mc__pos__control__main_8cpp_source.html), which has a member of type [PositionControl](https://px4.github.io/Firmware-Doxygen/d8/d5a/class_position_control.html) in [src/modules/mc_pos_control/PositionControl.cpp](https://px4.github.io/Firmware-Doxygen/df/dfe/_position_control_8cpp_source.html) which implements the core position and velocity controller as shown below:

```cpp
void PositionControl::_positionController()
{
	// P-position controller
	const Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(MPC_XY_P.get(), MPC_XY_P.get(), MPC_Z_P.get()));
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _constraints.speed_xy);
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}
```

The position controller is straightforward; it follows the above diagram/equations with the addition of speed constraints.  The speed constraint in $$z$$ is straightforward, however the constraint on horizontal (x-y plane) speed is more complicated.  The ```constrainXY``` utility function from ```src/modules/mc_pos_control/Utility/ControlMath.cpp``` basically scales the planar velocity such that the position feedback portion has higher priority than the velocity feedforward portion.

```cpp
void PositionControl::_velocityController(const float &dt)
{

	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame
	// - control output in D-direction has priority over NE-direction
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction is also limited by the maximum tilt.

	const Vector3f vel_err = _vel_sp - _vel;

	// Consider thrust in D-direction.
	float thrust_desired_D = MPC_Z_VEL_P.get() * vel_err(2) +  MPC_Z_VEL_D.get() * _vel_dot(2) + _thr_int(
					 2) - MPC_THR_HOVER.get();

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -MPC_THR_MIN.get();
	float uMin = -MPC_THR_MAX.get();

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * MPC_Z_VEL_I.get() * dt;

		// limit thrust integral
		_thr_int(2) = math::min(fabsf(_thr_int(2)), MPC_THR_MAX.get()) * math::sign(_thr_int(2));
	}

	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);
```

As shown above, the velocity controller is computed separately for the $$z$$ direction (here called *D-direction* for *Down*); this is done first to give attitude control priority, so if control in $$z$$ saturates (reaches the thrust limit) then control in the x-y plane is skipped.  Note PID control is computed using the position, velocity and acceleration states updated in the ```PositionControl``` object via the method ```updateState```; this gets called in the main ```run``` method in the class ```MulticopterPositionControl```, which first computes updated states in its method  ```set_vehicle_states```in which accelerations are computed numerically.  Note that there is no desired acceleration here, so the derivative term is a so-called *pure damping* term.  

The parameter ```MPC_THR_HOVER``` is the thrust required to hover, in other words a feedforward thrust added to the PID term to compensate for the weight of the drone.  This appears to be set automatically when transition from manual mode, assuming the user has the drone hovering before transitioning to a position control mode.

Note again that since control is performed in the NED frame, thrust limits get swapped; these limits are used to determine if [anti-windup](https://en.wikipedia.org/wiki/Integral_windup) must be used.  Briefly, integrator "windup" occurs when there's a large change in setpoint, causing a huge amount of error to be integrated which leads to overhshoot since it takes a long time to "integrate away" the large integrated error.  If the integrator is not wound-up, the error is integrated using the gain ```MPC_Z_VEL_I``` and the PID thrust is finally limited to the allowable range.

```cpp
	if (fabsf(_thr_sp(0)) + fabsf(_thr_sp(1))  > FLT_EPSILON) {
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else {
		// PID-velocity controller for NE-direction.
		Vector2f thrust_desired_NE;
		thrust_desired_NE(0) = MPC_XY_VEL_P.get() * vel_err(0) + MPC_XY_VEL_D.get() * _vel_dot(0) + _thr_int(0);
		thrust_desired_NE(1) = MPC_XY_VEL_P.get() * vel_err(1) + MPC_XY_VEL_D.get() * _vel_dot(1) + _thr_int(1);

		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(MPC_THR_MAX.get() * MPC_THR_MAX.get() - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in NE-direction.
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
		}

		// Get the direction of (r-y) in NE-direction.
		float direction_NE = Vector2f(vel_err) * Vector2f(_vel_sp);

		// Apply Anti-Windup in NE-direction.
		bool stop_integral_NE = (thrust_desired_NE * thrust_desired_NE >= thrust_max_NE * thrust_max_NE &&
					 direction_NE >= 0.0f);

		if (!stop_integral_NE) {
			_thr_int(0) += vel_err(0) * MPC_XY_VEL_I.get() * dt;
			_thr_int(1) += vel_err(1) * MPC_XY_VEL_I.get() * dt;

			// magnitude of thrust integral can never exceed maximum throttle in NE
			float integral_mag_NE = Vector2f(_thr_int).length();

			if (integral_mag_NE > 0.0f && integral_mag_NE > thrust_max_NE) {
				_thr_int(0) = _thr_int(0) / integral_mag_NE * thrust_max_NE;
				_thr_int(1) = _thr_int(1) / integral_mag_NE * thrust_max_NE;
			}
		}
	}
}
```

The x-y (forward-lateral) portion of the velocity control is done after the vertical portion to give priority to altitude control, and since the amount of lateral thrust available is a function of the vertical thrust and tilt angle.  Thus, the magnitude of thrust for control in the NE directions is limited to the minimum of the max allowable thrust based on tilt and based on leftover thrust after $$z$$ control (need to look into this a little more).  Anti-windup is again applied, the integrated error thrust is updated and limited by the max thrust.  This completes the velocity control portion of the position controller.

### A Note on Vehicle Constraints

Note  that the "constraints" used in the controllers above (eg  ```_constraints.tilt``` which limits lateral control) and elsewhere in the PX4 codebase are of type ```vehicle_constraints_s``` are updated in the ```PositionControl``` object via the method ```updateConstraints```; this gets called in the main ```run``` function of the class ```MulticopterPositionControl``` using constraints obtained earlier from the generic flight task interface class ```FlightTasks``` method ```getConstraints```.  The main thing to know is that these constraints are **global constraints** specifying true min/max values (eg a *physical* tilt max of 90*) which the task can and should overwrite to be more restrictive based on the demands of that task.

## Multicopter Attitude Controller

The attitude controller is more complex than the position/velocity controller because it involves quaternion math which can be difficult to parse at first glance.  The controller is based on the attitude controller of [this paper from ETH Zurich](../assets/pdf/px4_attitude_control.pdf).  Since it's unlikely I'll be using an attitude controller (and even less likely I'll be using an attitude rate or angular velocity controller aka *acro mode*), this is lower priority. At first glance, the controller appears to be a typical orientation control which computes a desired angular velocity from the logarithm of the error quaternion between desired and estimated orientations. It drops the yaw for this portion, adding yaw control back in at the end based on a feed-forward yaw rate command from the user.

$$
\Delta q = q_{est}^{-1}\otimes q_{des}
$$

is the error quaternion rotating from estimated to desired orientation, and the angular velocity command is something like

$$
\omega_{cmd} \propto \mbox{log}(\Delta q)
$$

where the command involves a gain, rate checks etc.

```cpp
void
MulticopterAttitudeControl::control_attitude()
{
	vehicle_attitude_setpoint_poll();
	_thrust_sp = _v_att_sp.thrust;

	/* prepare yaw weight from the ratio between roll/pitch and yaw gains */
	Vector3f attitude_gain = _attitude_p;
	const float roll_pitch_gain = (attitude_gain(0) + attitude_gain(1)) / 2.f;
	const float yaw_w = math::constrain(attitude_gain(2) / roll_pitch_gain, 0.f, 1.f);
	attitude_gain(2) = roll_pitch_gain;

	/* get estimated and desired vehicle attitude */
	Quatf q(_v_att.q);
	Quatf qd(_v_att_sp.q_d);

	/* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
	q.normalize();
	qd.normalize();

	/* calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch */
	Vector3f e_z = q.dcm_z();
	Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (abs(qd_red(1)) > (1.f - 1e-5f) || abs(qd_red(2)) > (1.f - 1e-5f)) {
		/* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		 * full attitude control anyways generates no yaw input and directly takes the combination of
		 * roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable. */
		qd_red = qd;

	} else {
		/* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
		qd_red *= q;
	}

	/* mix full and reduced desired attitude */
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix *= math::signNoZero(q_mix(0));
	/* catch numerical problems with the domain of acosf and asinf */
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(yaw_w * acosf(q_mix(0))), 0, 0, sinf(yaw_w * asinf(q_mix(3))));

	/* quaternion attitude control law, qe is rotation from q to qd */
	Quatf qe = q.inversed() * qd;

	/* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	 * also taking care of the antipodal unit quaternion ambiguity */
	Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

	/* calculate angular rates setpoint */
	_rates_sp = eq.emult(attitude_gain);

	/* Feed forward the yaw setpoint rate.
	 * yaw_sp_move_rate is the feed forward commanded rotation around the world z-axis,
	 * but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	 * Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	 * and multiply it by the yaw setpoint rate (yaw_sp_move_rate).
	 * This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	 * such that it can be added to the rates setpoint.
	 */
	_rates_sp += q.inversed().dcm_z() * _v_att_sp.yaw_sp_move_rate;


	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_auto_rate_max(i), _auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_mc_rate_max(i), _mc_rate_max(i));
		}
	}
}
```

The yaw attitude gain is set to the average of the roll and pitch attitude gains, while the yaw weight is computed as the original yaw attitude gain scaled by the average roll/pitch gain and constrained between zero and one.