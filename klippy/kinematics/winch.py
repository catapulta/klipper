# Code for handling the kinematics of cable winch robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import stepper
import mathutil
import logging
import can
import cantools
import numpy as np
from scipy.optimize import linprog, minimize
from typing import List, Union, Tuple

class AutoCalibrate:
    # https://github.com/ElinaChhabra/Trilateration-in-3d/blob/master/trilateration.py
    # https://stackoverflow.com/questions/1406375/finding-intersection-points-between-3-spheres
    # https://ciis.lcsr.jhu.edu/lib/exe/fetch.php?media=courses:446:2016:446-2016-08:algebraicmultilaterationnorrdine.pdf

    @staticmethod
    def multilat_to_dist_err(point, anchors, line_lengths):
        # point of size 3 dim
        # line_lengths of number of anchors dim
        return ((np.linalg.norm(point - anchors, axis=1) - np.abs(line_lengths))**2).sum()

    @staticmethod
    def multilaterate_error(self, anchors, line_lengths):
        point = minimize(self.multilat_to_dist_err,
                        x0=np.ones(3)*50,
                        args=(anchors, line_lengths))
        if point.success:
            return AutoCalibrate.multilat_to_dist_err(point.x, anchors, line_lengths)
        else:
            return None

    @staticmethod
    def multilaterate_errors(x, obs_line_lengths, num_points_grid=None, num_points_torque=None):
        anchors = x[:4*3].reshape(4, 3)
        line_init = x[4*3:4*3+4]
        errors = []
        for obs_line_length in obs_line_lengths.T[:num_points_torque]:  # for every data point
            multilat_err = AutoCalibrate.multilaterate_error(anchors, obs_line_length + line_init)
            # skip if failure
            if multilat_err is None:
                continue
            errors.append(multilat_err)
        mean_error = np.array(errors).mean()
        return mean_error
    
    @staticmethod
    def _find_line_lengths(self, points, anchors):
        line_lengths = []
        for anchor in anchors:
            line_lengths.append(np.linalg.norm(points - anchor, axis=1))
        line_lengths = np.array(line_lengths)
        return line_lengths

    @staticmethod
    def euclidean(x, hat_line_lengths, measured_points, num_points_grid=None, num_points_torque=None):
        anchors = x[:4*3].reshape(4, 3)
        line_init = x[4*3:4*3+4]

        line_lengths = AutoCalibrate.find_line_lengths(measured_points[:num_points_grid], anchors[:, :num_points_grid])
        sqerror = (line_lengths - (hat_line_lengths[:, :num_points_grid] + line_init[:, None]))**2
        return sqerror.sum()  # sum across anchors and data points
    
    @staticmethod
    def euclidean_anchor(x, line_init, hat_line_lengths, measured_points, num_points_grid=None, num_points_torque=None):
        x = np.concatenate([x, line_init])
        return AutoCalibrate.euclidean(x, hat_line_lengths, measured_points, num_points_grid, num_points_torque)

    @staticmethod
    def euclidean_anchor_d(x, line_init, hat_line_lengths, measured_points, num_points_grid=None, num_points_torque=None):
        x = np.concatenate([x[:-1], line_init[:-1], x[-1:]])
        return AutoCalibrate.euclidean(x, hat_line_lengths, measured_points, num_points_grid, num_points_torque)

    @staticmethod
    def multi_err_and_euclidean(x, measured_line_lengths, measured_points, num_points_grid=None, num_points_torque=None):
        # basic sensitivity analysis

        # 1 in init = 5
        # 1 in anchors ABC = 2-5 xy, 0.1 y
        # 1 in anchor D = 0.1 xy, 5 y

        # 0.5 in init = 1.4
        # 0.5 in anchors ABC = 0.07-1 xy, 0.04 y
        # 0.5 in anchor D = 0.03 xy, 1 y

        # 0.1 in init = 0.05
        # 0.1 in anchors ABC = 0.003-0.05 xy, 0.002 y
        # 0.1 in anchor D = 0.001 xy, 0.05 y

        multi_err = AutoCalibrate.multilaterate_errors(x, measured_line_lengths[:, num_points_grid:], num_points_grid=None, num_points_torque=num_points_torque)
        euc = AutoCalibrate.euclidean(x, measured_line_lengths[:, :num_points_grid], measured_points[:num_points_grid], num_points_grid=num_points_grid, num_points_torque=None)
        return euc + multi_err

    @staticmethod
    def multi_err_and_euclidean_anchor(x, line_init, measured_line_lengths, measured_points, num_points_grid, num_points_torque):
        x = np.concatenate([x, line_init])
        return AutoCalibrate.inter_anchor_and_euclidean(x, measured_line_lengths, measured_points, num_points_grid, num_points_torque)

    @staticmethod
    def multi_err_and_euclidean_line_init(x, anchor, measured_line_lengths, measured_points, num_points_grid, num_points_torque):
        x = np.concatenate([anchor, x])
        return AutoCalibrate.inter_anchor_and_euclidean(x, measured_line_lengths, measured_points, num_points_grid, num_points_torque)

    @staticmethod
    def _simulate_points_sphere(self, n_points, radius, center=np.array([50, 50, 50])):
        def sample_sphere(n_points, radius):
            oversample_proportion = 1 + (4/3 * np.pi * radius**3) / (2 * radius)**3
            points = np.random.rand(int(n_points * oversample_proportion), 3) * 2 * radius
            points -= [radius, radius, radius]
            accepted_points = points[np.linalg.norm(points, axis=1) < radius]
            return accepted_points
        points = sample_sphere(n_points, radius)[:n_points]
        while len(points) < n_points:
            new_points = sample_sphere(n_points - len(points), radius)
            points = np.concatenate([points, new_points], axis=0)
        points = points + center
        return points[:n_points]

    def _simulate(self,
                 anchors: Union[np.array, None] = None,
                 line_init: Union[np.array, None] = None, 
                 num_sim_points: int = 5000,
                 noise_mult: float = 3
    ) -> Tuple[np.array]:
        if anchors is None:
            anchors = np.array(
            [
                [0, 0, -10],
                [100, 0, -10],
                [50, 100, -10],
                [50, 50, 100]
                ]
            )
        if line_init is None:
            line_init = np.array([100, 200, 300, 100])

        measured_points = np.array([
                                    [50., 50., 0.], [70., 50., 0.], [50., 70., 0.], [30., 50., 0.], [50., 30., 0.],
                                    [70., 70., 0.], [30., 70., 0.], [30., 30., 0.], [70., 30., 0.],
                                    [50., 50., 50.], [70., 50., 50.], [50., 70., 50.], [30., 50., 15.], [50., 30., 15.],
                                    ])

        sim_measured_points = self._simulate_points_sphere(num_sim_points, radius=40, center=np.array([50, 50, 50]))
        grid_points = np.concatenate([measured_points, sim_measured_points], axis=0)

        measured_line_lengths = self._find_line_lengths(grid_points, anchors)
        measured_line_lengths -= line_init[:, None]

        # pollute grid points
        grid_points[:measured_points.shape[0], :measured_points.shape[1]] += (np.random.rand(*measured_points.shape)-0.5)*2 * noise_mult*4.5e-5 # encoder precision is 4.5e-5

        return grid_points, line_init, measured_line_lengths

    def calibrate(self, grid_points: np.ndarray, measured_line_lengths: np.ndarray, anchors: np.ndarray, calibrate_all: bool = False):

        num_points_grid = 5
        num_points_torque = 100
        method = 'BFGS'
        NUM_ITER = 10

        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

        options = {
                #    'Powell': {'gtol': 1e-10, 'disp': True, 'maxiter': 1e3},
                #    'SLSQP': {'disp': True, 'maxiter': 1e5},
                #    'Nelder-Mead': {'maxfev': 50000, 'return_all': True, 'maxiter': 1e3},
                'BFGS': {'gtol': 1e-5, 'disp': True, 'maxiter': 1e5},
                #    'CG': {'gtol': 1e-2, 'disp': True, 'maxiter': 1e5},
                }


        x0 = np.concatenate([anchors.flatten(), np.zeros(measured_line_lengths.shape[0])])

        # first round optimization optimizing for anchors + D length
        res = minimize(AutoCalibrate.euclidean_anchor_d, args=(x0[4*3:4*3+4], measured_line_lengths, grid_points, num_points_grid, num_points_torque),
                method=method,
                options=options[method],
                tol=1e-5,
                x0=np.concatenate([x0[:4*3], x0[-1:]])
        )
        x0 = np.concatenate([res.x[:-1], x0[4*3:4*3+4-1], res.x[-1:]])
        print('Anchors:')
        print(x0[:4*3].reshape(4,3))
        print('Line lengths:')
        print(x0[4*3:])

        if (metric:=AutoCalibrate.inter_anchor_and_euclidean(x0, measured_line_lengths, grid_points, num_points_grid, num_points_torque)) < 1:
            print('Calibration successful. Metric is:', metric)
        else:
            if not calibrate_all:
                raise ValueError(f'Anchor calibration. If you believe line lengths need calibration try with `calibrate_all`. Error metric is: {metric}')
            print('Imprecise anchor calibration. Metric is:', metric)

            i = 0
            x0_prev = x0.copy()
            while True:
                res = minimize(AutoCalibrate.euclidean, args=(measured_line_lengths, grid_points, num_points_grid, num_points_torque),
                    method=method,
                    options={**options[method],
                            'maxiter': 1e4,
                            'gtol': 1e-4},
                    tol=1e-5,
                    # give it a stochastic shake to dislodge from local minima
                    x0=np.concatenate([(x0_prev[:4*3] + (np.random.rand(4*3)-0.5)*2 * 40).flatten(),
                                        x0_prev[4*3:4*3+4]])
                )
                x0 = res.x
                print('Anchors:')
                print(x0[:4*3].reshape(4,3))
                print('Line lengths:')
                print(x0[4*3:])
                metric = AutoCalibrate.inter_anchor_and_euclidean(x0, measured_line_lengths, grid_points, num_points_grid, num_points_torque)
                if i == NUM_ITER:
                    raise ValueError(f'Calibration failed. Error metric is: {metric}')
                if metric < 1:
                    print('Calibration successful. Metric is:', metric)
                    break
                else:
                    print('Calibration failed, retrying. Metric is:', metric)
                i += 1

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
    def __init__(self,
                 weight: float,
                 min_tension: float,
                 max_tension: float,
                 extra_wire_length: List[float],
                 num_pulleys: List[int],
                 springK: List[float],
                 ) -> None:
        self.weight = weight
        self.min_tension = min_tension
        self.max_tension = max_tension
        self.extra_wire_length = extra_wire_length
        self.num_pulleys = num_pulleys
        self.springK = springK

    def _get_tension(self, position):
        # TODO: should manage error in computation
        return self._compute_static_forces(position)

    def _compute_static_forces(self, position):
        # returns static forces at point `position`
        v_dirs = self.anchors - position[:, None]
        u_vectors = v_dirs / np.sqrt(np.sum(v_dirs**2, axis=0))
        # print('u_vectors', u_vectors)

        c = np.ones([self.anchors.shape[1]])
        A_eq = u_vectors
        # print('A_ub\n', A_eq)
        # add gravity
        b_eq = np.array([0]*(self.anchors.shape[0]-1) + [self.weight])
        bounds = np.array([[self.min_tension]*self.anchors.shape[1], [self.max_tension]*self.anchors.shape[1]]).T

        res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds)

        # print('cost', res.fun)
        # print('lagrange\n', res.ineqlin)
        if res.success:
            return res.x
        else:
            raise ValueError(f'Optimization failed. {res.message}')
    
    def get_spring(self, position, distances):
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
        # TODO: what is relaxedSpringLengthsOrigin (above)?
        tensions = self._get_tension(position)

        spring_distances = []
        for num_pulleys, extra_wire_length, springK, tension, distance in zip(self.num_pulleys, self.extra_wire_length, self.springK, tensions, distances):
            spring_distance = springK * (distance * num_pulleys + extra_wire_length) * tension  # TODO: check spring equation
            spring_distances.append(spring_distance)
        return spring_distances

class ODrive3Manager:
    # * Step/dir performance improved! Dual axis step rates up to 250kHz have been tested
    # * Circular setpoint mode must be enabled when the step/dir interface is used.
    def __init__(self, config) -> None:
        self._odrive_canbus_iface = config.get_printer().get("odrive_canbus_interface", None)

        self.bus = can.Bus("vcan0", bustype="virtual")
        # bus = can.Bus(self._odrive_canbus_iface, bustype="socketcan")
        
        self._can_db = cantools.database.load_file("odrive-cansimple.dbc")

        self._axis_ids = config.get_printer().get("odrive_axis_ids", None)        
        self._torque_constants = config.get_printer().get("motor_torque_constants", None)
        self._spool_teeth = config.get_printer().get("spool_teeth", None)
        self._motor_teeth = config.get_printer().get("motor_teeth", None)
        self._num_pulleys = config.get_printer().get("num_pulleys", None)
        self._spool_radius = config.get_printer().get("spool_radius", None)

        if self._axis_ids is None:
            raise config.error("Parameter `odrive_axis_ids` is required.")
        elif not isinstance(self._axis_ids, list):
            raise config.error("Parameter `odrive_axis_ids` must be a list.")

        # TODO: clean up this mess
        self._axis = {}
        for (axis_id,
             torque_constant,
             spool_teeth,
             motor_teeth,
             num_pulleys,
             spool_radius) \
                in zip(self._axis_ids,
                       self._torque_constants,
                       self._spool_teeth,
                       self._motor_teeth,
                       self._num_pulleys,self._spool_radius):
            self.axis['axis'] = ODrive3Axis(
                axis_id=axis_id,
                torque_constant=torque_constant,
                spool_teeth=spool_teeth,
                motor_teeth=motor_teeth,
                num_pulleys=num_pulleys,
                spool_radius=spool_radius,
                connection=self
                )

class ODrive3Axis:
    # ODRIVE CANBUS methods
    # https://docs.odriverobotics.com/v/0.5.5/can-protocol.html#messages

    # ; Torque constants. These are required for reading motor forces from ODrives
    # ; They are the same values as is configured in the ODrives themselves (8.27/330 for motors in the standard HP4 BOM)
    # ;M666 C0.025061:0.025061:0.025061:0.025061

    def __init__(self,
                 axis_id: int,
                 torque_constant: float,
                 spool_teeth: int,
                 motor_teeth: int,
                 num_pulleys: float,
                 spool_radius: float,
                 connection: ODrive3Manager
                 ) -> None:

        self._connection = connection
        self.axis_id = axis_id
        self.velocity_limit
        self.current_limit

        self.position  # mm
        self.torque  # Nm
        self.motor_force  # N
        self.controller_mode
        self.input_mode

        self.traj_velocity_limit
        self.traj_accel_limit
        self.traj_decel_limit
        self.traj_inertia

        self.motor_torque_constant = torque_constant
        self.spool_teeth = float(spool_teeth)  # float to avoid accidental int div  255
        self.motor_teeth = float(motor_teeth)  # float to avoid accidental int div 20
        self.num_pulleys = num_pulleys
        self.spool_radius = spool_radius  # mm 75

    @staticmethod
    def _int_to_little_endian_hex(num):
        hex_num = hex(num)
        big = bytearray.fromhex(hex_num)
        big.reverse()
        little = ''.join(f"{n:02X}" for n in big)
        return little
    
    def full_calibrate(self):
        # calibration must be done without load
        logging.log(f"Requesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE (0x03) on: {str(self.axis_id)}")
        msg = self._connection.can_db.get_message_by_name('Set_Axis_State')
        data = msg.encode({'Axis_Requested_State': 0x03})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

        # Read messages infinitely and wait for the right ID to show up
        while True:
            msg = self._connection.bus.recv()
            if msg.arbitration_id == ((self.axis_id << 5) | self.db.get_message_by_name('Heartbeat').frame_id):
                current_state = self.db.decode_message('Heartbeat', msg.data)['Axis_State']
                if current_state == 0x1:
                    logging.info("\nAxis has returned to Idle state.")
                    break

    def set_current_limit(self, current_limit: float):
        # M906 E1400 I60             ; Set motor currents (mA) and increase idle current to 60%

        self.current_limit = current_limit

        # 0x00F Set Limits
        logging.log(f"Setting IQ (current) to {current_limit} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Limits', {'Velocity_Limit': self.velocity_limit, 'Current_Limit': self.current_limit})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_velocity_limit(self, velocity_limit: float):
        self.velocity_limit = velocity_limit

        # 0x00F Set Limits
        logging.log(f"Setting Velocity Limit to {velocity_limit} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Limits', {'Velocity_Limit': self.velocity_limit, 'Current_Limit': self.current_limit})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_traj_velocity_limit(self, traj_velocity_limit: float):
        # Limit for Trajectory Mode (?)
        self.traj_velocity_limit = traj_velocity_limit

        # 0x011 set traj vel limit
        logging.log(f"Setting Trajectory Velocity Limit to {traj_velocity_limit} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Traj_Vel_Limit', {'Traj_Vel_Limit': self.traj_velocity_limit})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_traj_accel_limit(self, traj_accel_limit: float):
        # Limit for Trajectory Mode (?)
        self.traj_accel_limit = traj_accel_limit

        # 0x012 set traj accel limit
        logging.log(f"Setting Trajectory Acceleration Limit to {traj_accel_limit} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Traj_Accel_Limits', {'Traj_Accel_Limit': self.traj_accel_limit, 'Traj_Decel_Limit': self.traj_decel_limit})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_traj_decel_limit(self, traj_decel_limit: float):
        # Limit for Trajectory Mode (?)
        self.traj_decel_limit = traj_decel_limit

        # 0x012 set traj accel limit
        logging.log(f"Setting Trajectory Deceleration Limit to {traj_decel_limit} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Traj_Accel_Limits', {'Traj_Accel_Limit': self.traj_accel_limit, 'Traj_Decel_Limit': self.traj_decel_limit})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_traj_inertia(self, traj_inertia: float):
        # https://docs.odriverobotics.com/v/0.5.4/control-modes.html
        # controller.config.inertia is a value which correlates
        # acceleration (in turns / sec^2) and motor torque.
        # It is 0 by default.
        # It is optional, but can improve response of your system if correctly tuned.
        # Keep in mind this will need to change with the load / mass of your system.

        # Limit for Trajectory Mode (?)
        self.traj_inertia = traj_inertia

        # 0x012 set traj accel limit
        logging.log(f"Setting Trajectory Intertia to {traj_inertia} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Traj_Inertia', {'Traj_Inertia': self.traj_inertia})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_position_gain(self, position_gain: float):
        # in trajectory mode, sets accel, decel aggressiveness
        self.position_gain = position_gain

        logging.log(f"Setting Position Gain (P) to {position_gain} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Pos_Gain', {'Pos_Gain': self.position_gain})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_linear_count(self, linear_count: float):
        # useful for resetting absolute position
        self.linear_count = linear_count

        logging.log(f"Setting Linear Count to {linear_count} on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Linear_Count', {'Position': self.linear_count})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_force(self, force: float):
        lineTension_N = force / self.num_pulleys
        spoolTorque_Nm = lineTension_N * self.spool_radius * 0.001
        motorTorque_Nm = spoolTorque_Nm * self.motor_teeth / self.spool_teeth
        self.motor_force = motorTorque_Nm
        self.set_torque(motorTorque_Nm)

    def set_position(self, position: float):
        # Torque_FF and Vel_FF: Set them to zero.
        # The feedforward terms are for advanced users,
        # for example if you happen to know what the static
        # torques on the joints will be from the jacobian,
        # then you can improve the control ferformance by sending
        # them as feedforward - this takes a lot of load off of the
        # position controller and allows it to react faster to actual disturbances
        # https://discourse.odriverobotics.com/t/can-set-input-pos-velff-and-torqueff-confusion/6878/2
        self.position = position

        logging.log(f"Setting Position on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Input_Pos', {'Input_Pos': self.position, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def set_torque(self, torque: float):
        self.torque = torque

        raise NotImplementedError('Be careful with torque sign')  # TODO: torque sign behavior unknown

        logging.log(f"Setting Torque on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Set_Input_Torque', {'Input_Torque': self.torque})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)
    
    def _commit_controller_and_input_modes(self, controller_mode: str, input_mode: str):
        # dangerous use of file since enumeration might change
        # ODrive/tools/fibre-tools/interface_generator.py can programmatically generate these
        # current list in ODrive/tools/odrive/enums.py
        
        # ODrive creator: step/dir should use INPUT_MODE_POS_FILTER
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
        odrive_controller_mode_map = dict(
            CONTROL_MODE_VOLTAGE_CONTROL             = 0,
            CONTROL_MODE_TORQUE_CONTROL              = 1,
            CONTROL_MODE_VELOCITY_CONTROL            = 2,
            CONTROL_MODE_POSITION_CONTROL            = 3,
            )
        controller_mode_to_commit = odrive_controller_mode_map[controller_mode]

        odrive_input_mode_map = dict(
            INPUT_MODE_INACTIVE                      = 0,
            INPUT_MODE_PASSTHROUGH                   = 1,
            INPUT_MODE_VEL_RAMP                      = 2,
            INPUT_MODE_POS_FILTER                    = 3,
            INPUT_MODE_MIX_CHANNELS                  = 4,
            INPUT_MODE_TRAP_TRAJ                     = 5,
            INPUT_MODE_TORQUE_RAMP                   = 6,
            INPUT_MODE_MIRROR                        = 7,
            INPUT_MODE_TUNING                        = 8,
            )
        input_mode_to_commit = odrive_input_mode_map[input_mode]

        data = self._connection._can_db.encode_message('Set_Controller_Mode', {'Control_Mode': controller_mode_to_commit,
                                                                               'Input_Mode': input_mode_to_commit})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def soft_set_input_mode(self, input_mode: str) -> None:
        logging.log(f"Setting Input Mode to f{input_mode} on: {str(self.axis_id)}")
        self.input_mode = input_mode

    def set_position_mode(self, controller_mode: str) -> None:
        logging.log(f"Setting Controller Mode to Position on: {str(self.axis_id)}")
        self.controller_mode = controller_mode
        self._commit_controller_and_input_modes(controller_mode='CONTROL_MODE_POSITION_CONTROL',
                                                input_mode=self.input_mode)

    def set_torque_mode(self, controller_mode: str) -> None:
        logging.log(f"Setting Controller Mode to Torque on: {str(self.axis_id)}")
        self.controller_mode = controller_mode
        self._commit_controller_and_input_modes(controller_mode='CONTROL_MODE_TORQUE_CONTROL',
                                                input_mode=self.input_mode)

    def get_current(self):
        logging.log(f"Requesting IQ (current) on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Get_Iq', {'Iq_Setpoint': 0.0, 'Iq_Measured': 0.0})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

        raise NotImplementedError('Missing message reception.')  # TODO: receive get messages
    
        # for msg in self._connection.bus:
        #     if msg.arbitration_id == 0x01 | axisID << 5:
        #         print("\nReceived Axis heartbeat message:")
        #         msg = self._connection.can_db.decode_message('Heartbeat', msg.data)
        #         print(msg)
        #         if msg['Axis_State'] == 0x8:
        #             print("Axis has entered closed loop")
        #         else:
        #             print("Axis failed to enter closed loop")
        #         break

    def get_position(self):
        logging.log(f"Requesting Positional Encoder Estimate on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Get_Encoder_Estimates', {'Pos_Estimate': 0.0, 'Vel_Estimate': 0.0})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

        raise NotImplementedError('Missing message reception.')  # TODO: receive get messages

    def get_force(self):
        # computed by means of current estimation
        # // This force calculation if very rough, assuming perfect data from ODrive,
        # // perfect transmission between motor gear and spool gear,
        # // the exact same line buildup on spool as we have at the origin,
        # // and no losses from any of the bearings or eyelets in the motion system.
        motor_current = self.get_current()
        motorTorque_Nm = motor_current * self.motor_torque_constant
        # if (driver.boardAddress == 40 || driver.boardAddress == 41) // Driver direction is not stored on main board!! (will be in the future)
        # {
        #     motorTorque_Nm = -motorTorque_Nm;
        # }
        # TODO: be careful about torque sign
        # TODO: should we multiply radius?
        lineTension = 1000.0 * (motorTorque_Nm * (self.spool_teeth / self.motor_teeth)) / self.spool_radius
        force = lineTension * self.num_pulleys
        return force

    def reboot_odrive(self):
        logging.log(f"Requesting ODrive Reboot on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Reboot', {})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

    def clear_errors(self):
        logging.log(f"Requesting Error Clear on: {str(self.axis_id)}")
        data = self._connection._can_db.encode_message('Clear_Errors', {})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)
        self._connection.bus.send(msg)

class WinchKinematics(Spring):
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
        # TODO: possibly broken function, will only work if first 3 axis are parallel
        # to the print bed, since it discards the bottom point in trilat

        # TODO: function needs to take into account buildup and static forces

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
