# Elevator
An elevator is a simple mechanism that moves something up or down. This lesson will introduce some basic concepts in controls by helping you through programming an elevator.

---
## Why an Elevator?

Controlling elevators is simple because they have only one degree of freedom and are remarkably consistent. The same amount of power applied to a motor should almost always produce the exact same motion from the elevator.

This makes them easier to control compared to something like an arm, where the force of gravity changes depending on the angle of the arm (imagine hanging your arm straight down vs. horizontal, parallel to the ground: more force is needed to maintain the angle when your arm is horizontal).

---

1. First, install the required software. Good instructions on this can be found in [zero to robot](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html). For java programming, you will need to [install the game tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html) and then [install WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
	<br>
2. Start a new project. As of the 2025 season, Team 401 started with the AdvantageKit Swerve Drive template. Since a drivetrain isn't part of this project, download the Skeleton template from the latest release under [AdvantageKit releases](https://github.com/Mechanical-Advantage/AdvantageKit/releases)  (be careful to scroll past the 2027 alphas if you're reading this before 2027). If you're interested in how to setup AdvantageKit, a library we use for logging and odometry, check out the [installation docs](https://docs.advantagekit.org/getting-started/installation/).

	When using the Skeleton template, you'll need to create a RobotContainer.java file. [The RobotContainer file from the Swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/diff_drive/src/main/java/frc/robot/RobotContainer.java) is a good starting point. You can delete everything that creates or uses the Drive class. You can come back and do this at the end when it's time to instantiate the Elevator. In a real robot project, this file is added once and then whenever a new subsystem is added, its instantiation gets added to this file.
	<br>
3. Write your IO:
	There are a few fields we want to track in our inputs:

	- We need some way to measure the current position of the elevator. Our hypothetical elevator has an encoder on the spool, which turns a certain amount every time the elevator moves. For the sake of example, let's say this elevator moves 5 inches per rotation of the spool.
		*(An encoder is a sensor that measures an angle. See the glossary for more details)*
	- Whether or not that encoder is connected.
	- The setpoint position of the elevator.
	<br>