# Code for handling the kinematics of cable winch robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, mathutil

class AutoCalibrate:
    pass
class Spool:
    # Spool buildup=0.128181
    # Spool radii (including buildup when homed)

	# /* Naive buildup factor calculation (assumes cylindrical, straight line)
	#  * line diameter: 0.5 mm
	#  * spool height: 8.0 mm
	#  * (line_cross_section_area)/(height*pi): ((0.5/2)*(0.5/2)*pi)/(8.0*pi) = 0.0078 mm
	#  * Default buildup factor for 0.50 mm FireLine: 0.0078
	#  * Default buildup factor for 0.39 mm FireLine: 0.00475
	#  * In practice you might want to compensate a bit more or a bit less */
	# constexpr float DefaultSpoolBuildupFactor = 0.007;
	# /* Measure and set spool radii with M669 to achieve better accuracy */
	# constexpr float DefaultSpoolRadii[4] = { 75.0, 75.0, 75.0, 75.0}; // HP4 default
    pass

    def buildup(self):
        # //// Line buildup compensation
        # float stepsPerUnitTimesRTmp[HANGPRINTER_AXES] = { 0.0 };
        # Platform& platform = reprap.GetPlatform(); // No const because we want to set drive steper per unit
        # for (size_t i = 0; i < HANGPRINTER_AXES; i++)
        # {
        # 	const uint8_t driver = platform.GetAxisDriversConfig(i).driverNumbers[0].localDriver; // Only supports single driver
        # 	bool dummy;
        # 	stepsPerUnitTimesRTmp[i] =
        # 		(
        # 			(float)(mechanicalAdvantage[i])
        # 			* fullStepsPerMotorRev[i]
        # 			* platform.GetMicrostepping(driver, dummy)
        # 			* spoolGearTeeth[i]
        # 		)
        # 		/ (2.0 * Pi * motorGearTeeth[i]);

        # 	k2[i] = -(float)(mechanicalAdvantage[i] * linesPerSpool[i]) * spoolBuildupFactor;
        # 	k0[i] = 2.0 * stepsPerUnitTimesRTmp[i] / k2[i];
        # 	spoolRadiiSq[i] = spoolRadii[i] * spoolRadii[i];

        # 	// Calculate the steps per unit that is correct at the origin
        # 	platform.SetDriveStepsPerUnit(i, stepsPerUnitTimesRTmp[i] / spoolRadii[i], 0);
        # }
        pass

class Spring:
    # ; Flex compensation
    # M666 W1.0                 ; Mover weighs 1 kg. Set to 0 to disable flex compensation.
    # M666 S20000.0             ; Spring constant (rough approximation) for Garda 1.1 mm line (unit N/m).
    #                           ; The real value is somewhere between 20k and 100k.
    #                           ; Lower value gives more flex compensation.
    # M666 I0.0:0.0:0.0:0.0     ; Min planned force in four directions (unit N).
    #                           ; This is a safety limit. Should affect only exceptional/wrong moves,
    #                           ; for example moves outside of the reachable volume.
    # M666 X70.0:70.0:70.0:70.0 ; Max planned force in four directions (unit N)
    #                           ; This is a safety limit. Will affect moves close to
    #                           ; the limits of the reachable volume.
    # M666 T10.0                ; Desired target force (unit N).
    #                           ; The flex compensation algorithm aims for at least
    #                           ; this amount of fource in the ABC line directions at all times.
    #                           ; It can be thought of as a minimum pre-tension value.
    #                           ; It's recommended to set it around 10 times higher
    #                           ; than your W (mover weight in kg) value.

    # ; Guy wire lengths. Needed for flex compenation.
    # ; Guy wires go between spool and final line roller.
    # ; If your spools are all mounted on the D-anchor, on the ceiling plate, then you're all good,
    # ; and you don't need to configure M666 Y values explicitly.
    # ; If your spools are not all on the D-anchor then you must measure guy wire
    # ; lengths and set them here.
    # ; If your spools are all mounted on their respective anchors, so that you have no guy wires,
    # ; then you should configure zeroed guy wire lengths M666 Y0.0:0.0:0.0:0.0.
    # ;M666 Y-1.0:-1.0:-1.0:-1.0

    def get_spring(self):
        # //// Flex compensation

        # // If no guy wire lengths are configured, assume a default configuration
        # // with all spools stationary located at the D anchor
        # if (guyWireLengths[A_AXIS] < 0.0F or
        #     guyWireLengths[B_AXIS] < 0.0F or
        #     guyWireLengths[C_AXIS] < 0.0F or
        #     guyWireLengths[D_AXIS] < 0.0F) {
        # 	guyWireLengths[A_AXIS] = hyp3(anchors[A_AXIS], anchors[D_AXIS]);
        # 	guyWireLengths[B_AXIS] = hyp3(anchors[B_AXIS], anchors[D_AXIS]);
        # 	guyWireLengths[C_AXIS] = hyp3(anchors[C_AXIS], anchors[D_AXIS]);
        # 	guyWireLengths[D_AXIS] = 0.0F;
        # }

        # for (size_t i{0}; i < HANGPRINTER_AXES; ++i) {
        # 	springKsOrigin[i] = SpringK(distancesOrigin[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
        # }
        # float constexpr origin[3] = { 0.0F, 0.0F, 0.0F };
        # StaticForces(origin, fOrigin);
        # for (size_t i{0}; i < HANGPRINTER_AXES; ++i) {
        # 	relaxedSpringLengthsOrigin[i] = distancesOrigin[i] - fOrigin[i] / (springKsOrigin[i] * mechanicalAdvantage[i]);
        # }
        pass

class ODrive3:
    # ODRIVE CANBUS methods
    # https://docs.odriverobotics.com/v/0.5.5/can-protocol.html#messages

    # ; Torque constants. These are required for reading motor forces from ODrives
    # ; They are the same values as is configured in the ODrives themselves (8.27/330 for motors in the standard HP4 BOM)
    # ;M666 C0.025061:0.025061:0.025061:0.025061


    def get_current(self):
        pass

    def set_current_limit(self):
        # 0x00F Set Limits
        pass

    def set_max_speed(self):
        # 0x011 set traj vel limit
        pass

    def set_accel_limit(self):
        # 0x012 set traj accel limit
        pass

    def set_inertia(self):
        # https://docs.odriverobotics.com/v/0.5.4/control-modes.html
        # controller.config.inertia is a value which correlates
        # acceleration (in turns / sec^2) and motor torque.
        # It is 0 by default.
        # It is optional, but can improve response of your system if correctly tuned.
        # Keep in mind this will need to change with the load / mass of your system.
        pass

    def set_torque(self):
        pass

    def get_force(self):
        # // This force calculation if very rough, assuming perfect data from ODrive,
        # // perfect transmission between motor gear and spool gear,
        # // the exact same line buildup on spool as we have at the origin,
        # // and no losses from any of the bearings or eyelets in the motion system.
        # float motorTorque_Nm = motorCurrent.value * torqueConstants_[boardIndex];
        # if (driver.boardAddress == 40 || driver.boardAddress == 41) // Driver direction is not stored on main board!! (will be in the future)
        # {
        #     motorTorque_Nm = -motorTorque_Nm;
        # }
        # float const lineTension = 1000.0 * (motorTorque_Nm * (spoolGearTeeth_[boardIndex]/motorGearTeeth_[boardIndex])) / spoolRadii_[boardIndex];
        # float const force = lineTension * mechanicalAdvantage_[boardIndex];

        # mechanicalAdvantage_ is the pulley system (eg 2,2,2,4)
        self.get_current()
        pass

    def set_force(self):
        # float const lineTension_N = force_Newton / mechanicalAdvantage_[boardIndex];
		# float const spoolTorque_Nm = lineTension_N * spoolRadii_[boardIndex] * 0.001;
		# float motorTorque_Nm = spoolTorque_Nm * motorGearTeeth_[boardIndex] / spoolGearTeeth_[boardIndex];
        self.set_torque()
        pass

    def get_position(self):
        # MSG_GET_ENCODER_ESTIMATES
        pass

    def get_current(self):
        # MSG_GET_IQ
        pass

    def set_position(self):
        pass

    def set_torque_mode(self):
        self._set_controller_modes()
        pass

    def set_position_mode(self):
        self._set_controller_modes()
        pass

    def set_current(self):
        # ; Currents
        # M906 E1400 I60             ; Set motor currents (mA) and increase idle current to 60%
        pass

    def calibrate(self):
        # calibration must be done without load
        # AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        pass
    
    def _set_controller_mode(self):
        # CONTROL_MODE_TORQUE_CONTROL
        pass
    
    def _set_controller_input_mode(self):
        # 0x00B Controller Modes
        # step/dir should use INPUT_MODE_POS_FILTER
        # https://docs.odriverobotics.com/v/0.5.5/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode

        # input_filter_bandwidth: Float32Property
        # The desired bandwidth for INPUT_MODE_POS_FILTER.
        # Sets the position filter’s P and I gains to emulate a critically-damped 2nd order mass-spring-damper motion.
        # https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html?highlight=pos_filter

        # controller.config.input_mode = INPUT_MODE_POS_FILTER
        # controller.config.input_filter_bandwidth = <tuning: 50-300 or so>
        # controller.config.inertia = <inertia> [Nm/(turn/s^2))]
        # FWIW Vez3D got better performance with input filter than with direct step/dir even without any inertia FF (config.inertia = 0), so it's certianly worth trying instead of raw step dir (INPUT_MODE_PASSTHROUGH) 
        # https://discord.com/channels/742749488543957033/760849239134699591/895775300229660722
        # tobben thinks it's worse? https://discord.com/channels/742749488543957033/760849239134699591/895778522117832804

        pass

class WinchKinematics:
    def __init__(self, toolhead, config):
        # Setup steppers at each anchor
        self.steppers = []
        self.anchors = []
        for i in range(26):
            name = 'stepper_' + chr(ord('a') + i)
            if i >= 3 and not config.has_section(name):
                break
            stepper_config = config.getsection(name)
            s = stepper.PrinterStepper(stepper_config)
            self.steppers.append(s)
            a = tuple([stepper_config.getfloat('anchor_' + n) for n in 'xyz'])
            self.anchors.append(a)
            s.setup_itersolve('winch_stepper_alloc', *a)
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        acoords = list(zip(*self.anchors))
        self.axes_min = toolhead.Coord(*[min(a) for a in acoords], e=0.)  # TODO: set min
        self.axes_max = toolhead.Coord(*[max(a) for a in acoords], e=0.)  # TODO: set max
        self.set_position([0., 0., 0.], ())
        
        # https://www.klipper3d.org/Code_Overview.html
        # for error handling
        # raise config.error("my error")

        # register_event_handler (to execute stuff once things are klippy:connect or klippy:ready )
        # will be useful for odrive calibration routine (perhaps this can't be done every time?? motors would pull each other)
        # config.get_printer().register_event_handler("klippy:connect", self.handle_connect)


        # arg0 is the command to be registered
        # arg1 is the name of the parameter to use on the console, e.g. "SET_FAN_SPEED FAN=value_here
        # arg2 is the value to set
        # arg3 is the method to call
        # arg4 is the help text
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_mux_command("SET_DIGIPOT", "DIGIPOT", self.name,
                                   self.cmd_SET_DIGIPOT,
                                   desc=self.cmd_SET_DIGIPOT_help)
        


    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        # Use only first three steppers to calculate cartesian position
        pos = [stepper_positions[rail.get_name()] for rail in self.steppers[:3]]
        return mathutil.trilateration(self.anchors[:3], [sp*sp for sp in pos])
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
    def home(self, homing_state):
        # XXX - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])
    def check_move(self, move):
        # XXX - boundary checks and speed limits not implemented
        pass
    def get_status(self, eventtime):
        # XXX - homed_checks and rail limits not implemented
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return WinchKinematics(toolhead, config)
