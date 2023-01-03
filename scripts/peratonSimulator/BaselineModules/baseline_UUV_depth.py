'''Peraton Labs Design Submission'''
from audioop import avg
from operator import truediv
import numpy as np
import pymap3d.utils as coords
import csv
import os
import math
import pandas as pd
from scipy.optimize import fsolve
# from pyspline.pyCurve import Curve
import matplotlib.pyplot as plt
import scipy.interpolate as sci
import GPPlanning


class uuv:
    """UUV model class."""

    """Initialization"""

    def __init__(self, z_control):
        self.vehicleMass = 1221.87  # [kg]
        self.initialVolume = 1.189  # [m^3]
        self.ballastMass = (1030 * self.initialVolume) - self.vehicleMass
        self.ballastVolume = self.ballastMass / 1030  # m^3
        self.initialMass = self.vehicleMass + self.ballastMass
        self.batteryCap = 168  # [Whr]

        self.tradeoff = 0.5131827

        wps = np.array(
            [-0.0440566060897208, -0.0248240698620003, -0.107037692767652, -0.0219444642741428, -0.026825331751674,
             -0.0593671277427065, -0.02420744721168, -0.139899000380344, -0.05121187599767, -0.0205029830056119,
             -0.00603013441440597,
             -0.0251546268034846, -0.065016295425312, -0.032817561895604, -0.201382652390223, -0.01464347671504,
             -0.114590494075103])

        delta = np.array([0.056376715, 0.062458538, 0.004275523, 0.037468322, 0.015695149,
                          -0.073516116, 0.005871452, 0.046592754, 0.049167087, 0.021494145, 0.015998495, 0.01695275,
                          -0.051999475, 0.070228969,
                          0.017897519, -0.056531397, -0.068477254])

        delta = np.array(
            [1.91931391e-02, 2.81394271e-02, -1.49609075e-02, -7.42794112e-02, 9.09433151e-03, 2.90712709e-03,
             5.91784053e-03, 2.84221936e-02, 2.86322722e-02, 5.85993683e-02, 1.32261491e-02, 6.82011201e-02,
             1.70863633e-02, -1.86479213e-02,
             3.38859725e-02, 1.01303293e-02, 2.35361573e-02])

        self.sensorList = {
            'GPS': {
                # The GPS only works at the surface with no ice coverage.
                # Assume that these conditions are only met at the start and end of the challenge.

                # Trident Sensors Active GPS Antenna
                'accuracy': 0.003,  # [km] ~3m accuracy
                'hotelPower': 0.396,  # [W]
                'on': True
            },
            'INS': {  # update numbers, assume 0.05 from corpus
                'accuracyNoDVL': 0.009,  # [km/hr] from worst case Phins Compact C7
                'accuracyDVL': 0.009 * 0.05e-2,  # [km/hr] from worst case Phins Compact C7
                'hotelPower': 20,  # [W]
            },
            'AcousticComms': {
                # High Tech Inc (HTI-04-PCA/ULF) Acoustic navigation beacon hydrophone
                'range': 6.0,  # [km]
                'accuracy': 0.05,  # [km]
                'hotelPower': 11.8,  # [W]
                'on': True
            },
            'Altimeter': {
                'accuracy': 0.0024,  # [m] with Cg and Cb available
                'hotelPower': 5.5  # [W]
            },
            'DVL': {
                # Teledyne Tasman DVL - 300kHz Bottom Tracking
                'maxAltitude': 0.275,  # [km] - 275 m
                'accuracy': 0.0216,  # [km/hr] - 0.6 cm/s
                'hotelPower': 11.8,  # [W]
                'on': True
            },
            'CTD': {
                # Seabird SBE 61 Deep Argo CTD
                'depthAccuracy': 0.0001,  # [km] - 4.5 dbar/7km
                'densityAccuracy': 0.1,  # [kg/m^3] - 0.0002 S/m, 0.001 C, 4.5 dbar
                'hotelPower': 0.42  # [W]
            },
            'TowedArray': {
                # High Tech Inc (HTI-04-PCA/ULF) Acoustic navigation beacon hydrophone
                'detectionRange': 20,  # 20,  # [km]
                'accuracy': 0.1,  # [km]
                'hotelPower': 0,
                'diameter': 0.0075  # m
            }
        }

        self._sensor_power = 0
        for sensor in self.sensorList.keys():
            if 'hotelPower' in self.sensorList[sensor].keys():
                self._sensor_power += self.sensorList[sensor]['hotelPower']
        self._current_waypoint = 0
        self._update_timestamp = 0  # [hr]
        self._acceptable_range = 0.25  # [km]
        self.drop_payload = False
        self.done_drop = False

        self.z_control = z_control
        delta = np.zeros_like(wps)
        self.log = []
        nav_file = pd.read_csv('mission_loop.csv')
        x_coord, y_coord, depth_profile = nav_file['x_coord'], nav_file['y_coord'], np.array(nav_file['z_coord'])
        depth_profile[depth_profile > -0.05] = -0.05
        target = nav_file.index[nav_file['Label'] == 'target '].tolist()
        z_coord = np.zeros(len(x_coord))
        z_coord[0] = wps[0] + delta[0]
        z_coord[6] = wps[1] + delta[1]
        z_coord[8] = wps[2] + delta[2]
        z_coord[20] = wps[3] + delta[3]
        z_coord[26] = wps[4] + delta[4]
        z_coord[target] = -0.5  # from mission specs
        z_coord[28] = wps[5] + delta[5]
        z_coord[35] = wps[6] + delta[6]
        z_coord[41] = wps[7] + delta[7]
        z_coord[53] = wps[8] + delta[8]
        z_coord[60] = wps[9] + delta[9]
        z_coord[65] = wps[10] + delta[10]
        z_coord[72] = wps[11] + delta[11]
        z_coord[80] = wps[12] + delta[12]
        z_coord[100] = wps[13] + delta[13]
        z_coord[116] = wps[14] + delta[14]
        z_coord[130] = wps[15] + delta[15]
        z_coord[136] = wps[16] + delta[16]
        z_coord[-1] = z_coord[0]
        z_coord[z_coord > 0] = 0
        benchmarks = z_coord[z_coord < 0]
        targets = np.where(z_coord < 0)[0]
        slopes = np.zeros_like(z_coord)
        for i in range(len(targets) - 1):
            delta_y = benchmarks[i + 1] - benchmarks[i]
            delta_x = targets[i + 1] - targets[i]
            slopes[targets[i]:targets[i + 1]] = delta_y / delta_x

        z_full = z_coord
        z_last = 0
        slope = 0
        for i in range(len(z_coord)):
            if z_full[i] != 0:
                z_last = z_full[i]
                slope = 0
            if z_full[i] == 0:
                slope += slopes[i]
                z_full[i] = slope + z_last

        # plt.plot(z_full)
        # plt.plot(depth_profile)
        # plt.legend(['Path', 'Depth Profile'])
        # plt.show()
        # self.threeDtrack = Track(x_coord, y_coord, z_full, loop=True, depth=None)
        # self.threeDtrack.plot()
        x_coord = np.array(x_coord)[:, np.newaxis]
        y_coord = np.array(y_coord)[:, np.newaxis]
        data_points = np.concatenate((x_coord, y_coord), axis=1)
        self.scipy_threeDtrack = sci.RBFInterpolator(data_points, z_full, kernel='multiquadric', neighbors=1, epsilon=0,
                                                     degree=0)
        self.tracker = []

        self.pitchViolations = 0
        self.ballastViolations = 0

        self.refSpeeds = np.array([3, 4.5, 6])  # [km/hr] Reference speeds for UUV Coefficents
        self.refDrag = np.array([0.1365, 0.1298, 0.12553])  # found via OpenFoam
        self.refFrontArea = np.array([0.7])  # 1.227]) # [m^2] Frontal area of UUV
        self.propEfficiency = np.array(
            [0.77135637, 0.78763087, 0.791])  # found via OpenProp and our gear/motor matching algorithm
        self.pumpEfficiency = 0.78  # assumption from Jim Luby slides. We use one motor per tank and need max of 1.5 L/min (see file output)
        self.refAngles = np.array([-90, -50, -10, -5, 0, 5, 10, 50, 90])  # in degrees found via OpenFoam
        self.refCdDrag = np.array([2, 0.901, 0.14123596, 0.1351296, 0.1298, 0.1351296, 0.14123596, 0.901, 2])

        self._set_speed = 3.1  # [km/hr]
        self.length = 11.3  # m
        self.elements = 400
        self.sensorList['TowedArray']['detectionRange'] = self.elements * self.sensorList['TowedArray']['accuracy'] / 2

        self.posPitch = 66  # degrees, calculated in FreeCad
        self.negPitch = -66  # degrees, calculated in FreeCad
        self.minDensity = self.vehicleMass / self.initialVolume
        self.ballastMaxVol = 0.068  # m^3. calculated in FreeCad
        self.maxDensity = (self.ballastMaxVol * 1030 + self.vehicleMass) / self.initialVolume

        self.old_state = ([0, 0, 0, 0, 0, 0])
        if self.z_control:
            self.dt = 0.007  # need for context of ballasting and pitching
        else:
            self.dt = 0.08  # simulator will be much coarser grained too

        self.planner = GPPlanning.pyPlanning2D(True, 0.2, 0.001, 0.01, 0.2, 5)
        self.planner.buildMap(1, (0, 0), 50, 50)
        self.nav_list = []
        self.cnt = 5

    def towed_array_drag(self, rho, v):
        c_f = 0.00277
        c = 1500  # m/s
        BeaconFrequency = 2000  # Hz
        d = self.sensorList['TowedArray']['diameter']
        lamda = c / BeaconFrequency
        array_length = (self.elements - 1) * lamda / 2
        Drag = 0.5 * rho * (v ** 1.931) * math.pi * d * array_length * c_f + 2.96 * (v ** 1.963)
        return Drag

    def deltaFromCurrentVal(self, currentValue, desiredValue):
        """Subtraction utility."""
        differenceMagnitude = np.abs(desiredValue - currentValue)
        differenceDirection = (-1) ** float(desiredValue < currentValue)
        return (differenceDirection * differenceMagnitude)

    def power_cost(self, speed, rho, sensor_power, ballastPower, AoA):
        """Compute the change in power."""
        total_power = 0
        total_power += sensor_power
        Cd = np.interp(speed, self.refSpeeds, self.refDrag)
        getAoADrag = sci.interp1d(self.refAngles, self.refCdDrag * Cd / 0.1298, kind='cubic')
        Cd_final = getAoADrag(AoA)
        vehicle_drag_force = 0.5 * rho * self.refFrontArea * ((speed * 0.277778) ** 2) * Cd_final  # [W] V * drag Force
        vehicle_drag_force += self.towed_array_drag(rho, speed * 0.277778)
        drag_power = abs(speed * 0.277778) * vehicle_drag_force
        batt_power = drag_power / np.interp(speed, self.refSpeeds, self.propEfficiency)
        total_power += batt_power
        total_power += ballastPower / (self.pumpEfficiency)
        # print("Sensor Power: ", sensor_power)
        # print("Ballast Power: ", ballastPower)
        # print("Propulsion Power: ", batt_power, speed)
        return (total_power[0])

    def distance(self, x0, x1, y0, y1):
        """Euclidean distance between two points."""
        return (np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2))

    def next_nav_point(self, mission_data):
        """Return next nav point position."""
        if self._current_waypoint >= len(mission_data['navigation']['position']):
            # go back to start if we run out of navigation points
            self._current_waypoint = 0
        return (mission_data['navigation']['position'][self._current_waypoint])

    def update_nav(self, posX, posY, posZ, tau, mission_data):
        """Update the waypoint information."""
        update_nav_position_flag = False
        payload = False
        # did we time out looking?
        _acceptable_duration = 100
        if tau - self._update_timestamp > _acceptable_duration:
            # find closest nav point from library of remaining points
            self._timed_out = True
            closest_distance = np.inf  # [km]
            closest_point = 0
            for nidx in np.arange(self._current_waypoint, len(mission_data['navigation']['position'])):
                tempX, tempY, tempZ = mission_data['navigation']['position'][nidx]
                distance = self.distance(tempX, posX, tempY, posY)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_point = nidx
            if closest_point > self._current_waypoint:
                self._current_waypoint = closest_point
            else:
                self._current_waypoint += 1
                self._update_timestamp = tau
            update_nav_position_flag = True

        # within the search time, let us see if we are close
        else:
            self._timed_out = False
            # close enough to the (or any) point?
            navX, navY, navZ = self.next_nav_point(mission_data)
            distance = self.distance(navX, posX, navY, posY)
            if mission_data['navigation']['type'][self._current_waypoint] == 'beacon':
                self._acceptable_range = self.sensorList['TowedArray']['detectionRange'] - 1
            elif mission_data['navigation']['type'][self._current_waypoint] == 'waypoint':
                self._acceptable_range = mission_data['waypoints']['waypointRangeDelta'] - 0.05
            elif mission_data['navigation']['type'][self._current_waypoint] == 'target':
                self._acceptable_range = mission_data['target']['dropRangeDelta'] - 0.05
                payload = True
            if distance < self._acceptable_range:
                self._update_timestamp = tau
                update_nav_position_flag = True
                if payload:
                    if abs(posZ - mission_data['target']['position'][2]) < mission_data['target']['dropDepthDelta']:
                        self.drop_payload = True
                self._current_waypoint += 1  # move on to next one

        return (update_nav_position_flag)

    def execute(self, stateVariables, tau, mission_data=None, sensor_data=None):
        # unpack the variables
        vX, vY, vZ, energy, mass, volume, dropPayload, navPointNum = stateVariables

        # INS and CTD and Altimeter are always on
        insX, insY, insZ = sensor_data['INS']
        arrayX, arrayY = sensor_data['AcousticComms']
        depth, density = sensor_data['CTD']
        altitude = sensor_data['Altimeter']
        gps_x, gps_y = sensor_data['GPS']
        dvl_x, dvl_y, velZ = sensor_data['DVL']

        if depth is not None:
            posZ = depth
        else:
            posZ = insZ

        sensor_power = self._sensor_power

        ocean_depth = altitude + posZ  # NEGATIVE NUMBER
        if abs(ocean_depth - posZ) > (self.sensorList['DVL']['maxAltitude'] + 0.05):  # uncertainity
            self.sensorList['DVL']['on'] = False
            sensor_power -= self.sensorList['DVL']['hotelPower']
        else:
            self.sensorList['DVL']['on'] = True

        if tau > 1:
            self.sensorList['GPS']['on'] = False
            sensor_power -= self.sensorList['GPS']['hotelPower']
        else:
            self.sensorList['GPS']['on'] = True

        dist = np.zeros(len(mission_data['waypoints']['position']))
        for i in range(len(dist)):
            coord = mission_data['waypoints']['position'][i]
            dist[i] = (self.distance(coord[0], insX, coord[1], insY))

        if np.min(dist) < self.sensorList['AcousticComms']['range'] + 0.05:  # uncertainty
            self.sensorList['AcousticComms']['on'] = True
            sensor_power -= self.sensorList['AcousticComms']['hotelPower']
        else:
            self.sensorList['AcousticComms']['on'] = False

        if gps_x is not None and self.sensorList['GPS']['on']:
            posX = gps_x
            posY = gps_y
        elif arrayX is not None and self.sensorList['AcousticComms']['on']:
            posX = arrayX
            posY = arrayY
        else:
            posX = insX
            posY = insY

        if dvl_x is not None and self.sensorList['DVL']['on']:
            self.old_state = np.array([posX, posY, posZ, dvl_x, dvl_y, velZ])
        else:
            vx = (posX - self.old_state[0]) / self.dt
            vy = (posY - self.old_state[1]) / self.dt
            velZ = (posZ - self.old_state[2]) / self.dt
            self.old_state = np.array([posX, posY, posZ, vx, vy, velZ])

        update_flag = self.update_nav(posX, posY, posZ, tau, mission_data)
        delta_navPt = self._current_waypoint

        navX, navY, navZ = self.next_nav_point(mission_data)

        if self.z_control:
            # Set z-control
            # pts, s = self.threeDtrack.get_closest_pts(np.array([posX, posY, posZ]))
            # pts = self.scipy_threeDtrack(np.array([[float(posX), float(posY)]]))
            # self.tracker.append(pts)

            # print("SCIPY    : ", pts)
            # print("PySpline: ", pts[2])
            # self.tracker.append(pts_scipy-pts[2])
            # navZ = pts

            # navZ = float(np.clip(navZ, ocean_depth + 0.005, -0.002))
            if len(self.nav_list) == 0:
                set_vel = self._set_speed * (np.array([navX - posX, navY - posY, navZ - posZ], np.dtype('float')) / np.sqrt(
                    (navX - posX) ** 2 + (navY - posY) ** 2 + (navZ - posZ) ** 2))

                total_step = int(np.sqrt((navX - posX) ** 2 + (navY - posY) ** 2 + (navZ - posZ) ** 2) / self._set_speed )

                poses = np.vstack((np.linspace(posX, navX, num=total_step),
                                   np.linspace(posY, navY, num=total_step),
                                   np.linspace(posZ, navZ, num=total_step) )).transpose()
                # poses = np.vstack((np.array([self.old_state[0], self.old_state[1], self.old_state[2]]),poses))
                vels = np.array([[set_vel[0], set_vel[1], set_vel[2]]] * total_step)
                # vels = np.vstack((np.array([vX, vY, vZ]), vels))

                # print(len(vel_GP))
                if len(vels) > 1:
                    planner3D = GPPlanning.pyPlanning3D(0.2, 0.002, 0.0002, 5)
                    origin = (min(posX, navX) - 5,
                              min(posY, navY) - 5,
                              ocean_depth - 0.005)
                    size = np.array([int(abs(posX-navX)) + 10,
                                     int(abs(posY-navY)) + 10,
                                     int(abs(max(posZ, navZ) - ocean_depth)*100 + 10)])
                    planner3D.buildMap(1, 0.001, origin, size[0], size[1], size[2], 5)
                    vels[0,0] = vX
                    vels[0,1] = vY
                    vels[0,2] = vZ

                    self.nav_list = planner3D.optimize(poses, vels, self.dt*500)
                    a = np.array(self.nav_list)
                    plt.plot(a[:,2],label = "optimized")
                    plt.plot(range(0,len(self.nav_list)), np.array([float(size[2]) * 0.001 + origin[2]] * len(self.nav_list)), label = "sea floor" )
                    plt.plot(poses[:,2],'--', label = "origin")
                    plt.legend()
                    plt.xlabel("UUV sub waypoint")
                    plt.ylabel("depth (km)")
                    plt.savefig("1.png")
                    plt.close()

                else:
                    self.nav_list = poses.tolist()
                    self.cnt = 0
            if len(self.nav_list) > 0:
                navX = self.nav_list[0][0]
                navY = self.nav_list[0][1]
                navZ = self.nav_list[0][2]
                self.nav_list.pop(0)
                # if self.cnt >int(1/self.dt - 1) :
                #     self.nav_list.pop(0)
                #     self.cnt = 0
                # else:
                #     self.cnt = self.cnt + 1

            print(len(self.nav_list))
            # print(len(nav_GP), self._set_speed, total_step)
            # print("nav after: ", navX-bx, navY-by, navZ-bz)
            set_vel = self._set_speed * (np.array([navX - posX, navY - posY, navZ - posZ], np.dtype('float')) / np.sqrt(
                (navX - posX) ** 2 + (navY - posY) ** 2 + (navZ - posZ) ** 2))


            desired_vX = set_vel[0]
            desired_vY = set_vel[1]
            desired_vZ = set_vel[2]

            # desired_vX = set_vel[0]
            # desired_vY = set_vel[1]
            # desired_vZ = set_vel[2]
            speed = (desired_vX ** 2 + desired_vY ** 2 + desired_vZ ** 2) ** .5

            accelX = self.deltaFromCurrentVal(vX, desired_vX) / self.dt
            accelY = self.deltaFromCurrentVal(vY, desired_vY) / self.dt
            accelZ = self.deltaFromCurrentVal(vZ, desired_vZ) / self.dt
            vehicle_accel = accelZ

            gravity = 9.81  # m/s^2
            buoyant_accel = -1 * gravity * (
                        1 - density * (volume / mass))  # 1 kg/m^3 leads to ~ 1.3 km/hr in vel change
            buoyant_accel *= (60 ** 4) / 1000  # km/hr^2

            already_done = self.tradeoff * buoyant_accel
            accelZ += -1 * self.tradeoff * buoyant_accel  # what percent is absorbed into the pitch versus ballasting
            desired_vZ += accelZ * self.dt
            speed_new = (desired_vX ** 2 + desired_vY ** 2 + desired_vZ ** 2) ** .5
            speed = 0.5 * (speed_new + speed)  # we ramp up to speed over self.dt

            # ballasting to eliminate buoyancy effects
            def get_mass(var):
                new_mass, p_new = var[0], var[1]
                avg_mass = 0.5 * (mass + new_mass)
                p_avg = 0.5 * (p_new + density)
                buoyant_accel = (60 ** 4) / 1000 * -1 * gravity * (
                            1 - p_avg * (volume / avg_mass))  # standard SI units #NEGATIVE IS DOWN HERE
                buoyant_accel -= already_done  # km/hr
                z_new = posZ + velZ * self.dt + (accelZ + buoyant_accel) * self.dt ** 2  # kinematic equations of motion
                p_new = np.clip(-z_new * 20 / 4 + 1030, 1030, 1050)
                return [float(buoyant_accel), float(p_avg - 0.5 * (p_new + density))]

            # print(a.shape)
            new_mass, p_new = fsolve(get_mass, [float(mass + 1), 1030], xtol=1e-4, maxfev=100)
            deltaMass = new_mass - mass
            avg_mass = 0.5 * (new_mass + mass)
            p_avg = 0.5 * (density + p_new)
            addedVol = deltaMass / p_avg

            resize = False
            if self.ballastVolume + addedVol > self.ballastMaxVol:
                new_mass = self.ballastMaxVol * p_avg
                resize = True
                self.ballastViolations += 1
            elif self.ballastVolume + addedVol < 0:
                new_mass = self.vehicleMass
                resize = True
                self.ballastViolations += 1

            if resize:
                deltaMass = new_mass - mass
                avg_mass = 0.5 * (new_mass + mass)
                p_avg = 0.5 * (density + p_new)

            self.ballastVolume += deltaMass / p_avg
            Pressure = (navZ + posZ) * 1000  # in m
            ballastPower = abs(
                deltaMass / self.dt * gravity * Pressure) / 100  # Power  in W is weight of liquid in unit time * Head in meters

            buoyant_accel = -1 * gravity * (1 - p_avg * (volume / avg_mass)) * (60 ** 4) / 1000
            buoyant_accel -= already_done  # km/hr^2

            if resize:
                accelZ += -1 * buoyant_accel  # take care of what is left over finally
                desired_vZ += accelZ * self.dt
                speed_new = (desired_vX ** 2 + desired_vY ** 2 + desired_vZ ** 2) ** .5
                speed = 0.5 * (speed_new + speed)  # we ramp up to speed over self.dt

            buoyant_speed = np.array([0, 0, float(buoyant_accel * self.dt)])

            vehicle_speed = 0.5 * (np.array([self.old_state[3], self.old_state[4], self.old_state[5]]) +
                                   np.array([float(desired_vX), float(desired_vY), float(desired_vZ)]))
            vec_angle = math.asin(float(np.dot(buoyant_speed, vehicle_speed)) / float(
                np.linalg.norm(buoyant_speed) * np.linalg.norm(vehicle_speed)))
            self.AoA = math.degrees(vec_angle)

            pitch_angle = math.degrees(math.atan(desired_vZ / np.sqrt(desired_vX ** 2 + desired_vY ** 2)))
            if pitch_angle < self.negPitch or pitch_angle > self.posPitch:
                self.pitchViolations += 1

            deltaVolume = 0

        else:
            if len(self.nav_list) == 0:

                set_vel = self._set_speed * (
                        np.array([navX - posX, navY - posY]) / np.sqrt((navX - posX) ** 2 + (navY - posY) ** 2))

                total_step = int(np.sqrt((navX - posX) ** 2 + (navY - posY) ** 2) / self._set_speed )

                poses = np.vstack((np.linspace(posX, navX, num=total_step),
                               np.linspace(posY, navY, num=total_step),
                               np.zeros(total_step))).transpose()
                # poses = np.vstack((np.array([self.old_state[0], self.old_state[1], 0]),poses))
                vels = np.array([[set_vel[0], set_vel[1], 0]] * total_step)
                if len(vels >1):
                    vels[0,0] = vX
                    vels[0,1] = vY
                    self.nav_list = self.planner.optimize(poses[1:], vels, self.dt * 12.5)
                else:
                    self.nav_list = poses
                self.cnt = 0
            if len(self.nav_list) >0:

                navX = self.nav_list[0][0]
                navY = self.nav_list[0][1]
                if self.cnt >12:
                    self.nav_list.pop(0)
                    self.cnt = 0
                else:
                    self.cnt = self.cnt + 1
            set_vel = self._set_speed * (
                    np.array([navX - posX, navY - posY]) / np.sqrt((navX - posX) ** 2 + (navY - posY) ** 2))
            desired_vX = set_vel[0]
            desired_vY = set_vel[1]
            # print("set, modified, ", navX - self.nav_list[-1][0], navY - self.nav_list[-1][1] )
            print(len(self.nav_list))
            desired_vZ = 0
            accelX = self.deltaFromCurrentVal(vX, desired_vX) / self.dt
            accelY = self.deltaFromCurrentVal(vY, desired_vY) / self.dt
            accelZ = 0
            deltaMass = 0
            deltaVolume = 0
            speed = np.linalg.norm([(desired_vX, desired_vY)])
            ballastPower = 0
            self.AoA = 0

        # drop payload  
        delta_payload = 0
        if self.drop_payload and not self.done_drop:
            delta_payload = 10
            self.done_drop = True

        if self.z_control:
            self.log.append(
                [tau, vehicle_accel, buoyant_accel, deltaMass, deltaMass / p_avg, mass / volume, self.drop_payload,
                 self.AoA, pitch_angle])

        # power change
        power = -1 * self.power_cost(speed, density, sensor_power, ballastPower, self.AoA)

        # stop if finished
        if dropPayload >= 1:
            startX, startY, startZ = mission_data['start']['position']
            close_enough = self.distance(startX, posX, startY, posY) < mission_data['start']['finalRangeDelta']
            if close_enough:  # come to a stop
                accelX = -vX / self.dt
                accelY = -vY / self.dt
                accelZ = -vZ / self.dt

        retVector = [accelX, accelY, accelZ, power, deltaMass, deltaVolume, delta_payload, delta_navPt]
        return (retVector, self.dt)

    def terminate(self):
        data = pd.DataFrame(self.log)
        data.to_csv('path/BuoyancyConsiderations.csv')
        return self.pitchViolations, self.z_control, self.ballastViolations, self.tracker


class Track:
    def __init__(self, x_coord=None, y_coord=None, z_coord=None, loop=False, depth=None):

        self.loop = loop
        if z_coord is not None:
            # self.center_line = Curve(x=x_coord, y=y_coord, z=z_coord, k=4)
            self.threeD = True
        else:
            # self.center_line = Curve(x=x_coord, y=y_coord, k=4)
            self.threeD = False

        if depth is not None:
            # self.depth = Curve(x=x_coord, y=y_coord, z=depth, k=4)
            self.depthLength = self.depth.getLength()
        else:
            self.depth = None

        self.length = self.center_line.getLength()
        self.track_center = None

    def _interp_s(self, s):
        '''
      Given a list of s (progress since start), return corresponing (x,y,z) points
      on the track. 
      '''
        s = np.array([s])
        interp_pt = self.center_line.getValue(s)

        return interp_pt.T

    def interp(self, theta_list):
        '''
      Given a list of theta (progress since start), return corresponing (x,y)
      points on the track. In addition, return slope of trangent line on those
      points.
      '''
        if self.loop:
            s = np.remainder(theta_list, self.length) / self.length
        else:
            s = np.array(theta_list) / self.length
            s[s > 1] = 1
        return self._interp_s(s)

    def get_closest_pts(self, points):
        '''
      Points have [2xn] shape
      '''
        s, _ = self.center_line.projectPoint(points.T, eps=1e-3)
        closest_pt = self._interp_s(s)

        return closest_pt, s * self.length

    def project_point(self, point):
        s, _ = self.center_line.projectPoint(point, eps=1e-3)
        return s * self.length

    def interp_depth(self, theta_sample):
        s = np.array(theta_sample) / self.depthLength
        s[s > 1] = 1
        s = np.array([s])
        interp_pt = self.depth.getValue(s)
        return interp_pt

    def plot(self):
        N = 500

        if self.track_center is None:
            theta_sample = np.linspace(0, 1, N, endpoint=False) * self.length
            interp_pt = self.interp(theta_sample)
            self.track_center = interp_pt.T

        if self.depth is not None:
            theta_sample = np.linspace(0, 1, N, endpoint=False) * self.depthLength
            interp_pt = self.interp_depth(theta_sample)
            self.depth_profile = interp_pt.T

        if self.threeD:
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.plot(self.track_center[:, 0], self.track_center[:, 1], self.track_center[:, 2], 'b--')
            if self.depth is not None:
                ax.plot(self.depth_profile[:, 0], self.depth_profile[:, 1], self.depth_profile[:, 2], 'b')
            plt.show()
        else:
            plt.plot(self.track_center[:, 0], self.track_center[:, 1], color='blue')
            plt.show()
