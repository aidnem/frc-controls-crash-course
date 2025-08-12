Definitions of commonly used terms, and some helpful links, because lots of great resources already exist.

[WPILib Controls Glossary](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controls-glossary.html) - Great for general controls terms and concepts.

- PID Control
	- [Intro to PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html) 
	- [PID Controls in WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)
- Feedforward
	- [Intro to Feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html)
- IO
	An IO interface is a way of separating logic so that the code for simulation and real life is as similar as possible.

	For a great explanation of IO interfaces and why they're useful, checkout the [AdvantageKit IO interface docs](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/)
-  Robot.java and RobotContainer.java
	Robot.java is like the "main" function of a robot project. It controls the initial setup of very basic features like the logger, and its robotPeriodic function is the main loop. However, since 99% of logic should be handled within individual subsystems, Robot.java should be as minimal as possible. In robotPeriodic, the only action that should be taken is calling `CommandScheduler.getInstance().run()` (and optionally increasing thread priority to help the periodic function run faster).
	Instead, a RobotContainer should be instantiated in the Robot constructor. This RobotContainer is responsible for creating instances of all subsystems, registering commands, and adding button bindings.