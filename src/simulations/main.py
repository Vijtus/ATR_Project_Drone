import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from pid import PIDController

# Implementation based mostly on equations from this paper: https://browse.arxiv.org/pdf/2005.09858.pdf

class Quadcopter:
    def __init__(self, mass, Jx, Jy, Jz, l, C, g=9.81):
        """
        The constructor initializes a Quadcopter object with physical parameters and state.

        Parameters:
        - mass: The mass of the quadcopter.
        - Jx, Jy, Jz: Moments of inertia around the x, y, and z axes, respectively.
        - l: Arm length, the distance from the center of mass to the propellers.
        - C: Force-to-moment scaling factor.
        - g: Gravitational acceleration (default is 9.81 m/s^2).

        The state is initialized with zeros representing the starting conditions of position, orientation, and their respective rates.
        """
        # Quadcopter parameters
        self.mass = mass    # Mass
        self.Jx = Jx        # Moments of inertia
        self.Jy = Jy
        self.Jz = Jz
        self.l = l          # The distance from the center to each of the arms (used for torque calculations)
        self.C = C          # Force-to-moment scaling factor (used to convert between the force produced by the rotors and the resulting moment (or torque) about the center of the quadcopter)
        self.g = g          # Acceleration due to gravity

        # Initial state [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]
        self.state = np.zeros(12)

        self.last_control_inputs = np.zeros(4)

    def equations_of_motion(self, state, u):
        """
        Calculates the derivatives of the state vector based on the current state and control inputs.

        Parameters:
        - state: The current state vector [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot].
        - u: The control input vector [u1, u2, u3, u4] corresponding to total thrust and angular moments.

        Returns:
        - A numpy array containing the derivatives of the state vector, which includes the translational and rotational accelerations.
        """
        # Unpack the state
        x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot = state
        u1, u2, u3, u4 = u
        self.last_control_inputs = u

        # Equations of motion
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sin_psi = np.sin(psi)
        cos_psi = np.cos(psi)

        # Translational accelerations
        x_ddot = u1 * (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) / self.mass
        y_ddot = u1 * (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) / self.mass
        z_ddot = (u1 * (cos_phi * cos_theta) - self.mass * self.g) / self.mass

        # Rotational accelerations
        phi_ddot = u2 * self.l / self.Jx
        theta_ddot = u3 * self.l / self.Jy
        psi_ddot = u4 / self.Jz

        return np.array([x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot])

    def rk4_step(self, u, dt):
        """
        Performs a single step of the Runge-Kutta 4th order method to integrate the equations of motion.

        Parameters:
        - u: The control input vector for the current step.
        - dt: The time step size for integration.

        Updates the current state of the quadcopter by applying the RK4 integration method.
        """
        # Runge-Kutta 4th order method to update the state
        k1 = self.equations_of_motion(self.state, u)
        k2 = self.equations_of_motion(self.state + dt/2 * k1, u)
        k3 = self.equations_of_motion(self.state + dt/2 * k2, u)
        k4 = self.equations_of_motion(self.state + dt * k3, u)

        self.state += dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    def simulate(self, u, dt, n_steps):
        """
        Simulates the quadcopter motion over a series of time steps.

        Parameters:
        - u: The control input vector to apply during simulation.
        - dt: The time step size for simulation.
        - n_steps: The number of steps to simulate.

        The function updates the state of the quadcopter for each time step.
        """
        # Simulate the quadcopter's flight for n_steps
        for _ in range(n_steps):
            self.rk4_step(u, dt)
            # Here you we can store the state history if needed

    def calculate_motor_thrusts(self, u1, u2, u3, u4):
        """
        Calculates individual motor thrusts based on control inputs.

        Parameters:
        - u1, u2, u3, u4: The control inputs corresponding to total thrust and the roll, pitch, and yaw moments.

        Returns:
        - An array containing the computed motor thrusts necessary to achieve the desired control inputs.
        """
        # Unpack the quadcopter parameters needed to compute the thrusts
        m = self.mass
        Jx = self.Jx
        Jy = self.Jy
        Jz = self.Jz
        C = self.C

        # Control matrix A
        A = np.array([
            [1/m, 1/m, 1/m, 1/m],
            [0, 1/Jx, 0, -1/Jx],
            [1/Jy, 0, -1/Jy, 0],
            [-C/Jz, C/Jz, C/Jz, -C/Jz]
        ])

        # Control inputs vector
        u = np.array([u1, u2, u3, u4])

        # Calculate the pseudo-inverse of the control matrix A
        A_pinv = np.linalg.pinv(A)

        # Calculate motor thrusts using the pseudo-inverse
        T = A_pinv @ u

        return T
    
    def get_imu_data(self):
        """
        Retrieves simulated Inertial Measurement Unit (IMU) data from the quadcopter.

        Returns:
        - A numpy array with 3-axis accelerations and 3-axis angular rates, incorporating both gravitational and linear accelerations.
        """
        # Extract the current state variables
        _, _, _, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot = self.state

        # Extract the current accelerations from the state derivatives
        state_dot = self.equations_of_motion(self.state, self.last_control_inputs)
        x_ddot, y_ddot, z_ddot = state_dot[6:9]

        # Compute the rotation matrix from inertial frame to body frame
        R = self.rotation_matrix(phi, theta, psi)

        # Gravitational acceleration in inertial frame
        gravity_inertial = np.array([0, 0, -self.g])

        # Calculate the acceleration in the body frame, including gravity
        # The gravity vector is negated because it's in the opposite direction of the accelerometer's Z axis
        gravity_body = -R @ gravity_inertial

        # The accelerometer reads the sum of drone's acceleration and the gravitational acceleration
        acc_x = x_ddot + gravity_body[0]
        acc_y = y_ddot + gravity_body[1]
        acc_z = z_ddot - gravity_body[2]  # We subtract because the accelerometer also reads acceleration due to gravity

        # Gyroscope readings in the body frame
        gyro_x = phi_dot
        gyro_y = theta_dot
        gyro_z = psi_dot

        return np.array([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])

    def rotation_matrix(self, phi, theta, psi):
        """
        Computes the rotation matrix for transforming vectors from the inertial frame to the body frame.

        Parameters:
        - phi, theta, psi: The roll, pitch, and yaw angles of the quadcopter.

        Returns:
        - A 3x3 rotation matrix based on the ZYX Euler angle convention.
        """
        # Define the rotation matrices
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])

        R_y = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

        R_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        # The final rotation matrix
        R = R_z @ R_y @ R_x
        return R

mass = 1.0  # kg
Jx = Jy = Jz = 0.01  # kg*m^2
l = 0.5  # m
C = 1.0  # Force-to-moment scaling factor

quad = Quadcopter(mass, Jx, Jy, Jz, l, C)

# PID gains
kp, ki, kd = 0.001, 10.0, 0.0

# PID controllers for each axis
pid_roll = PIDController(kp, ki, kd)
pid_pitch = PIDController(kp, ki, kd)
pid_yaw = PIDController(kp, ki, kd)

# Time settings
total_time = 30.0  # Total time to simulate
dt = 0.01  # Time step
n_steps = int(total_time / dt)  # Number of steps to simulate

# Example: angular_velocity_setpoints.append([roll_setpoint, pitch_setpoint, yaw_setpoint])
angular_velocity_setpoints = []  # Define your angular velocity setpoints here
for t in range(n_steps):
    if t < n_steps / 10:
        angular_velocity_setpoints.append([0.01, 0.0, 0.0])
    elif t < 2 * n_steps / 10:
        angular_velocity_setpoints.append([0.01, 0.0, 0.0])
    elif t < 3 * n_steps / 10:
        angular_velocity_setpoints.append([0.0, 0.0, 0,0])
    elif t < 4 * n_steps / 10:
        angular_velocity_setpoints.append([0.0, 0.0, 0.0])
    elif t < 5 * n_steps / 10:
        angular_velocity_setpoints.append([-0.01, 0.0, 0.0])
    elif t < 6 * n_steps / 10:
        angular_velocity_setpoints.append([-0.01, 0.0, 0.0])
    elif t < 7 * n_steps / 10:
        angular_velocity_setpoints.append([0.0, 0.0, 0.0])
    elif t < 8 * n_steps / 10:
        angular_velocity_setpoints.append([0.0, 0.0, 0.0])
    elif t < 9 * n_steps / 10:
        angular_velocity_setpoints.append([0.0, 0.0, 0.0])
    else:
        angular_velocity_setpoints.append([0.0, 0.0, 0.0]) 


# Initialize an empty list to store motor thrusts
motor_thrusts_history = []

# Simulate and collect state history for plotting
state_history = []
imu_data_history = []
valocity_history = []
angular_velocity_history = []
t = 0
for t in range(n_steps):

    # Get IMU readings
    imu_data = quad.get_imu_data()
    imu_data_history.append(imu_data)
    current_roll_velocity = imu_data[3]
    current_pitch_velocity = imu_data[4]
    current_yaw_velocity = imu_data[5]

    # Update PID controllers
    u_roll = pid_roll.update(angular_velocity_setpoints[t][0], current_roll_velocity, dt)
    u_pitch = pid_pitch.update(angular_velocity_setpoints[t][1], current_pitch_velocity, dt)
    u_yaw = pid_yaw.update(angular_velocity_setpoints[t][2], current_yaw_velocity, dt)

    # Combine the control inputs
    u = [quad.mass * quad.g * 1.01, u_roll, u_pitch, u_yaw]  # Total thrust and angular moments

    quad.simulate(u, dt, 1)
    state_history.append(quad.state.copy())  # Copy the current state to history

    t += 1

    motor_thrusts = quad.calculate_motor_thrusts(*u)
    # # print the thrusts for debugging
    # print(motor_thrusts)
    # # find the largest difference between the thrusts
    # print(max(motor_thrusts) - min(motor_thrusts))
    # print the acceleration on Z axis and the time
    # print only every second
    if t % 100 == 0:
        print(imu_data[2], t * dt)
    motor_thrusts_history.append(motor_thrusts)

    # append the velocity and angular velocity
    valocity_history.append([quad.state[6], quad.state[7], quad.state[8]])
    angular_velocity_history.append([quad.state[9], quad.state[10], quad.state[11]])



# Convert history to a numpy arrays for ease of plotting

motor_thrusts_history = np.array(motor_thrusts_history)
state_history = np.array(state_history)
imu_data_history = np.array(imu_data_history)
valocity_history = np.array(valocity_history)
angular_velocity_history = np.array(angular_velocity_history)

# Plotting
fig, ax = plt.subplots(1, 4, figsize=(20, 5))

# Position
ax[0].plot(state_history[:, 0], state_history[:, 1], label='XY Trajectory')
ax[0].set_xlabel('X Position')
ax[0].set_ylabel('Y Position')
ax[0].set_title('XY Plane Trajectory')
ax[0].legend()

# Altitude
ax[1].plot(np.arange(0, total_time, dt), state_history[:, 2], label='Altitude')
ax[1].set_xlabel('Time')
ax[1].set_ylabel('Z Position')
ax[1].set_title('Altitude over Time')
ax[1].legend()

# Angles
ax[2].plot(np.arange(0, total_time, dt), state_history[:, 3], label='Roll (phi)')
ax[2].plot(np.arange(0, total_time, dt), state_history[:, 4], label='Pitch (theta)')
ax[2].plot(np.arange(0, total_time, dt), state_history[:, 5], label='Yaw (psi)')
ax[2].set_xlabel('Time')
ax[2].set_ylabel('Angle (rad)')
ax[2].set_title('Angles over Time')
ax[2].legend()

# Motor thrusts
ax[3].plot(np.arange(0, total_time, dt), motor_thrusts_history[:, 0], label='Motor 1 Thrust')
ax[3].plot(np.arange(0, total_time, dt), motor_thrusts_history[:, 1], label='Motor 2 Thrust')
ax[3].plot(np.arange(0, total_time, dt), motor_thrusts_history[:, 2], label='Motor 3 Thrust')
ax[3].plot(np.arange(0, total_time, dt), motor_thrusts_history[:, 3], label='Motor 4 Thrust')
ax[3].set_xlabel('Time')
ax[3].set_ylabel('Thrust')
ax[3].set_title('Motor Thrusts over Time')
ax[3].legend()

plt.tight_layout()

fig, ax_imu = plt.subplots(2, 3, figsize=(10, 8))

ax_imu[0, 0].plot(np.arange(0, total_time, dt), imu_data_history[:, 0], label='X Acceleration')
ax_imu[0, 1].plot(np.arange(0, total_time, dt), imu_data_history[:, 1], label='Y Acceleration')
ax_imu[0, 2].plot(np.arange(0, total_time, dt), imu_data_history[:, 2], label='Z Acceleration')
ax_imu[1, 0].plot(np.arange(0, total_time, dt), imu_data_history[:, 3], label='X Angular Rate')
ax_imu[1, 1].plot(np.arange(0, total_time, dt), imu_data_history[:, 4], label='Y Angular Rate')
ax_imu[1, 2].plot(np.arange(0, total_time, dt), imu_data_history[:, 5], label='Z Angular Rate')

for i in range(3):
    ax_imu[0, i].set_xlabel('Time')
    ax_imu[0, i].set_ylabel('Acceleration (m/s^2)')
    ax_imu[0, i].legend()
    ax_imu[1, i].set_xlabel('Time')
    ax_imu[1, i].set_ylabel('Angular Rate (rad/s)')
    ax_imu[1, i].legend()


plt.tight_layout()

fig, ax_vel = plt.subplots(2, 3, figsize=(20, 5))

ax_vel[0, 0].plot(np.arange(0, total_time, dt), valocity_history[:, 0], label='X Velocity')
ax_vel[0, 1].plot(np.arange(0, total_time, dt), valocity_history[:, 1], label='Y Velocity')
ax_vel[0, 2].plot(np.arange(0, total_time, dt), valocity_history[:, 2], label='Z Velocity')
ax_vel[1, 0].plot(np.arange(0, total_time, dt), angular_velocity_history[:, 0], label='X Angular Velocity')
ax_vel[1, 1].plot(np.arange(0, total_time, dt), angular_velocity_history[:, 1], label='Y Angular Velocity')
ax_vel[1, 2].plot(np.arange(0, total_time, dt), angular_velocity_history[:, 2], label='Z Angular Velocity')

for i in range(3):
    ax_vel[0, i].set_xlabel('Time')
    ax_vel[0, i].set_ylabel('Velocity (m/s)')
    ax_vel[0, i].legend()
    ax_vel[1, i].set_xlabel('Time')
    ax_vel[1, i].set_ylabel('Angular Velocity (rad/s)')
    ax_vel[1, i].legend()

plt.tight_layout()

# Separate window for the 3D trajectory
fig_3d = plt.figure()
ax_3d = fig_3d.add_subplot(111, projection='3d')

# Create a color map based on time (in seconds)
time_colors = np.arange(state_history.shape[0]) / (1 / dt)

# 3D Trajectory with color gradient
scatter = ax_3d.scatter(state_history[:, 0], state_history[:, 1], state_history[:, 2], c=time_colors, cmap='viridis')

# Position colorbar at bottom
# Set the aspect of the colorbar and pad to adjust the colorbar size and space from the main plot
colorbar = fig_3d.colorbar(scatter, ax=ax_3d, orientation='horizontal', shrink=0.6, aspect=30, pad=0.1)
colorbar.set_label('Time')

ax_3d.set_xlabel('X Position')
ax_3d.set_ylabel('Y Position')
ax_3d.set_zlabel('Z Position')
ax_3d.set_title('3D Trajectory with Time Gradient')

plt.show()
