# Advanced Topics in Robotics: Project Charter

### Group members:
- @Vijtus
- @diminDDL 


# Proposed project:
We propose to create a basic controller for a quadcopter. We already have a working and proven platform that works with existing firmware/controllers, and is easily modifiable.

Our primary goal is: a drone controller that allows for autonomous self-leveling and stabilization, making the task of controlling it by the pilot easier.

## Purpose and Justification
Obviously we are not solving new problems in a single semester project course, but we can justify what our project can inspire or assist with in the future if some of use decide to keep working on it as a thesis or personal project.

The end product, would be a cheap, easy to use and modify minimal drone controller. It can work as a great starting point for us and other people to experiment with low level drone development. (We intend to publicly release our work when we are done) The project will also help us exercise and gain a more practical experience with the topics we learned last year, such as control theory. 

The project can later be expanded to be a fully featured universal navigation computer for many types of arial vehicles, competing with existing solutions such as ArduPilot, INAV, Betaflight and so on.

## Objectives, Constraints and Success Criteria

### Objectives
- **Hardware**: Design and assemble a minimally viable control board including everything needed for our goal.
  To be more specific the controller needs:
  * A microcontroller board to run the control algorithm.
  * Interface connectors to connect to the RC receiver, outputs for the ESCs, and a debug port.
  * Battery voltage monitoring.
  * An IMU board in order to know the current position of the drone.

- **Software Stack**:
    * **HAL**: Implement a basic and portable hardware abstraction layer, simplifying the subsequent development stages.
    * **Control Algorithm**: Implement a standard drone leveling algorithm with easily adjustable parameters to simplify tuning.
    * **I/O**: Implement methods to receive and transmit additional data (excluding the control signals, because they are handled by the receiver that comes with the RC controller) between the ground and the drone, for information like battery SoC, telemetry and other system parameters.

- **Testing**: Test and verify the developed controller often and rigorously in order to fix any bugs or other problems.

## Success criteria
- **Completion of the hardware**: completing the hardware in any functional state is a success since it proves the concept. 
    * Time constraint: *3 weeks*, since many parts of the software can be tested on the existing hardware, the custom controller is not required for initial implementations of the higher level algorithms. If we fail to implement our own hardware we will use the existing controller currently installed on the drone and focus purely on software.

- **Achieve level flight with custom controller**: If the above controller flies and provides even rudimentary gyroscope stabilization, this is considered a success.
    * Time constraint: *8 weeks*, if rudimentary flight anc control cannot be achieved within this time (roughly half of the semester).


### Constraints
- **Safety**: The fast spinning propellers as well as the large lithium battery are a real hazard that needs to be mitigated and caution needs to be taken to ensure that even in the event of a catastrophic failure safety can be ensured.

- **Legality**: The drone platform is *~800g*, which puts it into the A1 class of the open category. This category requires the pilot to register and perform an online test in order to be allowed to fly it. Only 2 team members posses this certification. This also means that we need to ensure our testing stays within legal limits set by the category.

- **Time Constraints**: Balancing this project with academic commitments remains a primary concern, requiring good time management.

- **Scope Constraints**: It is important to remain in scope and not let feature creep occur since that can interfere with timely project completion. This is one of this is the reason why the project definition is relatively simple, in case we achieve the goal early, we will keep working on expanding the cope as necessary.

## Roles and Responsibilities

### @Vijtus
- **Project Manager**: As the director of the STYK student club, Wiktor has experience in team and project management with a good track record of delivering everything on time.

- **High level Software**: With knowledge in programming languages like Python and MATLAB, he plays a role in performing simulations and verifying concepts on a higher level before spending the time to implement in the embedded firmware.

- **Drone Pilot**: Wiktor has an active drone certification, making him a qualified pilot for testing rounds.

### @diminDDL
- **Hardware**: Dmytro's profound knowledge in electronics and mechanics make him the ideal candidate to design and integrate the hardware components of this project.

- **Embedded firmware Engineering**: Dmytro has 4 years of freelance experience working on and implementing custom firmware for constrained embedded systems in C and C++. This makes him the perfect candidate to implement the algorithms in an efficient and safe manner.

- **Drone Architect and Pilot**: Having built a drone from scratch, the firsthand experience provides an invaluable perspective. This ensures the project. He also has a drone certification and real experience dealing with things going wrong mid flight.

### Aliakasei Kvach
- **Theoretical Consultant**: Aliakasei's grounding in mathematics and robotics will be a valuable asset in rigorously defining the algorithms and approaches used in software.
- **Research Engineer**: With a concrete understanding of scientific literature Aliakasei can help by performing literature review and identifying the important components for our specific project, as well as communicating them efficiently to the rest of the team.
- **Mechanical Engineering**: Aliakasei would be responsible for the mechanical assembly and maintenance of the drone. As well as ensure safety of the system before flight.

## Potential Risks and Their Mitigation

**Risk**: The custom hardware does not work, this includes the accelerometer/not working/not being detected, the microcontroller not powering on/not being able to program it, the inability to receive inputs from the remote controller, or the inability to output control signals to the ESCs controlling the motors.

**Mitigation 1**: Dmytro's extensive experience in electronics allows him to troubleshoot and modify the problematic hardware in place and in a timely manner restoring at least partial functionality sufficient for the project.

**Mitigation 2**: Using the existing APM 2.8 controller, it is a proven platform that is known to work as a great drone controller.

**Risk**: Component failure after a crash landing taking the drone out of service, this can include propeller failure, frame failure, or even partial electronics failure.

**Mitigation**: Having a sufficient stock of spare parts that are prone to breaking such as propellers and frame components, specifically, we will keep several spare sets of propellers since they are the most likely component to fail. If something more exotic breaks, most of the components are generic and can be ordered and delivered within a few days.

**Risk**: Given the technical depth of the project, timely documentation and reporting might be challenging, leading to potential reporting lags, this can include the exclusion of up to date information from reports and presentations.

**Mitigation**: Allocating Alex to be responsible for documentation, as well as often team meetings to talk about our current progress and what should be documented.

**Risk**: With a sufficiently complex project like this software can become a bottleneck, for example the hardware can be ready for a new test run but the software is still stuck in an unflyable state.

**Mitigation**: If we do not manage to create a complete flight controller, we will pivot our project to focus on a smaller part of it, such as obstacle avoidance, built on top of an existing open source stack, such as ardupilot.

## Current plans
- Parts list:

| Part Category       | Specific Part                | Part Number   | Additional Info               |
|---------------------|------------------------------|---------------|-------------------------------|
| Main Controller     | Raspberry Pi Pico            | SC0915        | 133MHz (overclockable to 400MHz) ARM M0+ dual core microcontroller |
| RC & Receiver       | Flysky I6                    | FS-I6         | Includes RC and receiver      |
| IMU Sensor          | LSM6DSOX + LIS3MDL 9 DOF IMU | LSM6DSOX/LIS3MDL | Precise and low drift IMU  |
| Telemetry Module    | Generic Bluetooth Serial Module | N/A         | For real-time telemetry      |
| Drone Frame         | F450 Drone Frame             | F450          |                               |
| Battery             | 3S 5.2Ah Li-Po Pack          | N/A           |                               |
| Prototyping Board   | Prototyping Board            | N/A           | For mounting everything       |
| Passive Components  | Miscellaneous                | N/A           | Resistors, capacitors, etc.   |
