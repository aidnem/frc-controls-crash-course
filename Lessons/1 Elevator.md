# Elevator
An elevator is a simple mechanism that moves something up or down. This lesson will introduce some basic concepts in controls by helping you through programming an elevator.

## Why an Elevator?

Controlling elevators is simple because they have only one degree of freedom and are remarkably consistent. The same amount of power applied to a motor should almost always produce the exact same motion from the elevator.

This makes them easier to control compared to something like an arm, where the force of gravity changes depending on the angle of the arm (imagine hanging your arm straight down vs. horizontal, parallel to the ground: more force is needed to maintain the angle when your arm is horizontal).

## How to use this guide

This guide is intended to provide step-by-step help on the basics of writing a subsystem for FRC. Learning by doing is a great strategy, especially for FRC when combined with googling words as they come up. To aid in learning by doing, this guide tries to serve as a reference that can guide a new FRC programmer through the process, while taking detours to give quick explanations on terms that may be unfamiliar. 95% of this guide can be figured out by googling, looking at existing FRC code, and experimentation, but that can be daunting.

While this guide contains code snippets, it is *strongly encouraged* not to copy and paste them into your robot project: you will learn better by typing the code yourself, by gaining muscle memory, understanding the code better as you write it, and having to go back and fix errors that may appear from typing mistakes. In addition, some of the code in the guide may contain errors; nobody is perfect. If something doesn't work, try to fix it youself. If you do find an obvious mistake, feel free to open an issue on GitHub to fix it.

## Writing the Elevator IO

1. First, install the required software. Good instructions on this can be found in [zero to robot](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html). For java programming, you will need to [install the game tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html) and then [install WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
	<br>
2. Start a new project. As of the 2025 season, Team 401 started with the AdvantageKit Swerve Drive template. Since a drivetrain isn't part of this project, download the Skeleton template from the latest release under [AdvantageKit releases](https://github.com/Mechanical-Advantage/AdvantageKit/releases)  (be careful to scroll past the 2027 alphas if you're reading this before 2027). If you're interested in how to setup AdvantageKit, a library we use for logging and odometry, check out the [installation docs](https://docs.advantagekit.org/getting-started/installation/).

	When using the Skeleton template, you'll need to create a RobotContainer.java file. [The RobotContainer file from the Swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/diff_drive/src/main/java/frc/robot/RobotContainer.java) is a good starting point. You can delete everything that creates or uses the Drive class. You can come back and do this at the end when it's time to instantiate the Elevator. In a real robot project, this file is added once and then whenever a new subsystem is added, its instantiation gets added to this file.
	<br>
1. Create your IO:
	 The IO interface is a way to separate the logic of controlling motors or updating a simulation from the main robot logic. We'll define an Elevator IO interface and then create separate implementations for real and simulated IO.
	<br>
	Let's create an `ElevatorIO.java` file under `src/main/java/frc/robot/subsystems/elevator`. Create a public interface called `ElevatorIO`.

	VSCode should be able to automatically generate most imports as you type code, but if not, here are some imports that'll come in handy for this IO:
	
	```java
	import static edu.wpi.first.units.Units.Amps;
	import static edu.wpi.first.units.Units.Rotations;
	import static edu.wpi.first.units.Units.RotationsPerSecond;
	import static edu.wpi.first.units.Units.Volts;
	
	import edu.wpi.first.units.AngularAccelerationUnit;
	import edu.wpi.first.units.AngularVelocityUnit;
	import edu.wpi.first.units.VoltageUnit;
	import edu.wpi.first.units.measure.Angle;
	import edu.wpi.first.units.measure.AngularVelocity;
	import edu.wpi.first.units.measure.Current;
	import edu.wpi.first.units.measure.MutAngle;
	import edu.wpi.first.units.measure.MutAngularVelocity;
	import edu.wpi.first.units.measure.MutCurrent;
	import edu.wpi.first.units.measure.MutVoltage;
	import edu.wpi.first.units.measure.Per;
	import edu.wpi.first.units.measure.Voltage;
	import org.littletonrobotics.junction.AutoLog;
	```

	The ElevatorIO class will define a few methods for the reading input data and applying control output to the elevator. It will also define an enum to keep track of the current output mode: the elevator can either be in Closed-Loop mode (using PID control to automatically target its goal position), Current mode (used to manually apply a certain current, Current mode is used to manually tune gains for closed loop mode for TorqueCurrentFOC systems, which use PID control in terms of current), and Voltage mode (used to manually apply a certain amount of power. We'll use this later for homing the elevator).
		
	  ```java
	/** An override/output mode for the elevator */
	enum ElevatorOutputMode {
	  ClosedLoop, // Not overriding, standard operation
	  Current, // Overriding, manually applying a current
	  Voltage // Overriding, manually applying a voltage
	}
	
	// We define blank classes for inputs and outputs for now so that we can make the updateInputs and applyOutputs methods take them as arguments
	public static class ElevatorInputs {}
	public static class ElevatorOutputs {}
	
	/**
	  * Updates an ElevatorInputs with the current information from sensors readings and from the
	  * motors.
	  *
	  * @param inputs ElevatorInputs object to update with latest information
	  */
	public void updateInputs(ElevatorInputs inputs);
	
	/**
	  * Applies requests to motors and updates an ElevatorOutputs object with information about motor
	  * output.
	  *
	  * @param outputs ElevatorOutputs updated with latest applied voltage
	  */
	public void applyOutputs(ElevatorOutputs outputs);
	  
	/**
	  * Set the goal position of the CANCoder which the elevator will control to when it is not in
	  * override mode
	  *
	  * <p>This method should only be called by the ElevatorSubsystem! There is important safety
	  * control logic housed there which, if bypassed, will be sorely missed.
	  */
	public void setElevatorEncoderGoalPos(Angle goalPos);
	
	/** Get the absolute position of the 19 tooth CANCoder. */
	public Angle getElevatorEncoderAbsPos();
	
	/** Get the absolute position of the 17 tooth CANCoder. */
	public Angle getSmallCANCoderAbsPos();
	
	/**
	  * Set the position of the 19 tooth CANCoder. This position is separate from absolute position and
	  * can track multiple rotations.
	  */
	public void setElevatorEncoderPosition(Angle newAngle);
	
	/**
	  * Set the position of the 17 tooth CANCoder. This position is separate from absolute position and
	  * can track multiple rotations.
	  */
	public void setSmallCANCoderPosition(Angle newAngle);
	
	/**
	  * Set the override voltage for the elevator when in Voltage output mode
	  *
	  * @param volts The voltage to apply
	  */
	public void setOverrideVoltage(Voltage volts);
	
	/**
	  * Set the static current (because of FOC) that will be applied when the elevator is in Current
	  * output mode.
	  */
	public void setOverrideCurrent(Current current);
	
	/**
	  * Set whether the elevator should use ClosedLoop control (default), voltage override, or current
	  * override
	  */
	public void setOutputMode(ElevatorOutputMode mode);
	
	/** Update PID gains for the elevator */
	public void setPID(double p, double i, double d);
	
	/** Set profile constraints to be sent to Motion Magic Expo */
	public void setMaxProfile(
	  AngularVelocity maxVelocity,
	  Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
	  Per<VoltageUnit, AngularVelocityUnit> expo_kV);
	
	/** Set feedforward gains for closed-loop control */
	public void setFF(double kS, double kV, double kA, double kG);
	
	/** Set whether or not the motors should brake while idle */
	public void setBrakeMode(boolean brakeMode);
	
	/** Set the stator current limit for both elevator motors */
	public void setStatorCurrentLimit(Current currentLimit);
	
	/** Set whether or not the motors on the elevator should be disabled. */
	public void setMotorsDisabled(boolean disabled);
	```
	
	You will notice that most of the values aren't doubles; we use the [WPILib Units library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html). This allows us to keep track of what units are associated with values, instead of having to name every variable with the unit that it is in. For instance, a method taking an argument with type `Distance` can accept both `Meters.of(1.0)` and `Inches.of(12.0)`, and both will be handled correctly.
	<br>
1. Define the ElevatorInputs class:
	There are a few fields we want to track in our inputs:

	- We need some way to measure the current position of the elevator. Our hypothetical elevator has an encoder on the spool, which turns a certain amount every time the elevator moves. For the sake of example, let's say this elevator moves 5 inches per rotation of the spool.
		*(An encoder is a sensor that measures an angle. See the glossary for more details)*
		Because the IO level is only concerned with directly communicating with hardware, the IO will track the position of the elevator in terms of the angle of the encoder:
		```java
		/** The angle of the elevator spool encoder */
		public MutAngle encoderPos = Rotations.mutable(0.0);
		```

		Because the default behavior of measures is to create new objects whenever a different value is used, we use the `MutAngle` class instead. This allows us to update the value inside of the `encoderPos` variable instead of replacing it with a new object. This helps prevent using large amounts of memory and overwhelming the garbage collector with deleting all of our old unused measures. We have to use the `.mutable()` method of `Rotations` rather than the regular `.of()` syntax in order to create a mutable measure.
		
	- Whether or not that encoder is connected.
		```java
		public boolean encoderConnected = false; // Always set default values so that they're correct in their initial state. For instance, if we have yet to communicate with the encoder, we can assume it's not connected.
		```
	- The setpoint position of the elevator.
		Setpoint position is the current target position of the motion profile. This example will be using [Motion Magic](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html), a system that can run a motion profile and PID controller on the motor controller itself.
		```java
		public MutAngle encoderSetpointPos = Rotations.of(0.0);
		```
	- The supply and stator current of the elevator motors. For the sake of example, let's use 2 motors for this elevator.

		```java
		public MutCurrent elevatorLeadMotorStatorCurrent = Amps.mutable(0.0);
		public MutCurrent elevatorLeadMotorSupplyCurrent = Amps.mutable(0.0);
		public MutCurrent elevatorFollowerMotorStatorCurrent = Amps.mutable(0.0);
		public MutCurrent elevatorFollowerMotorSupplyCurrent = Amps.mutable(0.0);
		```
	- We can also log a few other fields. For instance, the current error and mechanism velocity reported by Motion Magic, and the angle of the motor.
		```java
		/**
	     * Current closed-loop error (distance from setpoint position) as reported by the lead motor
	     * TalonFX, in rotations.
	     */
		public double motionMagicError = 0.0;
	
	    /** Velocity of the mechanism, as reported by the lead motor TalonFX */
	    public MutAngularVelocity elevatorMechanismVelocity = RotationsPerSecond.mutable(0.0);
	
	    public MutAngle leadMotorAngle = Rotations.mutable(0.0);
		```
	
	Finally, set up logging for the class by annotating it with AdvantageKit's `@AutoLog` decorator. Next time you run a gradle build, this will auto-generate `[Class]AutoLogged` classes. For instance, annotating ElevatorInputs would generate a class called `ElevatorInputsAutoLogged`. This class extends the original class, but automatically outputs all of its data to the log file and to Network Tables.
	
	  ```java
	  @AutoLog
	  public static class ElevatorInputs { /*... */ }
	  ```
1. Define your ElevatorOutputs class.
	This is very similar to the ElevatorInputs class. Most teams don't use a separate Outputs class to log outputs, but since the Inputs class can get quite large and overwhelming. Because this guide is using onboard PID control running on the TalonFX controller itself, our robot code doesn't know the applied voltage to the motors without querying the motor controller for that data. This data is then stored in the ElevatorOutputs class, which is updated whenever applyOutputs is called. We also track the contributions of the P, I, and D terms of the PID controller individually, as this can be helpful for diagnosing the cause of tuning/overshoot issues. Most systems likely won't need these fields, and there's a case to be made for decreasing the number of fields that are logged, as logging 700+ fields can cause AdvantageScope to crash on slower laptops and overwhelm weak WiFi connections.
	*Note: Having an outputs class at all is a bit unconventional, and it's technically a misnomer, as the Outputs object actually contains data that, while related to the output of the motors, is an input of the robot programming. Team 401 may not even use Outputs classes in the 2026 season. If you are greatly offended by the concept, feel free to add the fields from here to ElevatorInputs, and then make the applyOutputs method update the ElevatorInputs object as well.*
	
	  ```java
	  @AutoLog
	  public static class ElevatorOutputs {
		/** Are the elevator motors currently disabled in software? */
		public boolean motorsDisabled = false;
	
		/** The current output mode of the elevator */
		public ElevatorOutputMode outputMode = ElevatorOutputMode.ClosedLoop;
	
		/** The voltage applied to the elevator motor */
		public MutVoltage elevatorAppliedVolts = Volts.mutable(0.0);
	
		/** Contribution of the p-term to motor output */
		public MutVoltage pContrib = Volts.mutable(0.0);
	
		/** Contribution of the i-term to motor output */
		public MutVoltage iContrib = Volts.mutable(0.0);
	
		/** Contribution of the d-term to motor output */
		public MutVoltage dContrib = Volts.mutable(0.0);
	  }	
	  ```

2. Now that the IO interface is defined, we can write a real life IO implementation according to the specification defined in our interface. We define the real IO first since, when using CTRE motors and encoders like the Kraken X60 and CANCoder, you can use [Phoenix 6](https://v6.docs.ctr-electronics.com/en/stable/), the library for controlling and communicating with these devices, to simulate the devices with the same handle that you would use to communicate with them in real life. This means that the Simulation IO can extend the real life IO and not have to duplicate nearly as much logic. More details on simulating CTRE devices can be found in the next step.

	The convention for creating an IO implementation is to either suffix the name with `Real` or the name of the motors/encoders that are used. Team 401 frequently uses `TalonFX` for IOs using CTRE motors, since [TalonFX](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html) is the motor controller to communicate with these motors.

    Create a file called `ElevatorIOTalonFX.java` or `ElevatorIOReal.java` in the folder with the interface definition. Create a class:

    ```java
    public class ElevatorIOTalonFX implements ElevatorIO {}
    ```

    The first thing this IO needs to do is keep track of its current goal position, override voltage, and override current, as well as the current output mode. Add those fields:

    ```java
    MutAngle encoderGoalAngle = Rotations.mutable(0.0);

    MutVoltage overrideVoltage = Rotations.mutable(0.0);
    MutCurrent overrideCurrent = Amps.mutable(0.0);

    ElevatorOutputMode outputMode = ElevatorOutputMode.ClosedLoop;

    boolean motorsDisabled = false;
    ```

    Next, we need a handle to interact with each motor and the encoder. For motors, this is a `TalonFX` object, and for CANcoders the class is called `CANcoder`. We also add a local `TalonFXConfiguration` object, which will hold the motor config that we will apply. Finally, we can include the control requests that we will send to the motor. These are objects that describes what behavior we want the motor to follow. We have a request for the closed-loop, voltage, and current output modes.

    ```java
    TalonFX leadMotor; // This motor will control to the goal position
    TalonFX followerMotor; // This motor will follow/copy the lead motor's commands

    CANcoder elevatorEncoder;

    // Reuse the same TalonFXConfiguration object to avoid creating new ones repeatedly and leaking memory
    TalonFXConfiguration talonFXConfigs;

    // Reuse the same requests to avoid the garbage collector having to clean up requests created every loop
    MotionMagicExpoTorqueCurrentFOC closedLoopRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    VoltageOut voltageOut = new VoltageOut(0.0);
    TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);
    ```

    `MotionMagicExpoTorqueCurrentFOC` is a crazy class name. Let's break it down:

    - `MotionMagicExpo` - This means that the request uses Motion Magic Expo, a motion profile that runs on the TalonFX controller. Expo comes from the fact that it's an exponential motion profile, which you can read about [here](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo).
    - `TorqueCurrentFOC` - A TorqueCurrentFOC request uses [Field Oriented Control](https://en.wikipedia.org/wiki/Vector_control_(motor)), also called vector control. This produces a slightly stronger peak output than standard voltage control. The TorqueCurrent part comes from the fact that this control method is in terms of acceleration/current; rather than commanding the motor to apply a certain voltage/power, it is commanded directly in terms of acceleration.

    Now that we've defined our member variables, we can write a constructor. As you write your constructor, add each Constant that you use to an ElevatorConstants.java file as public, final values. Separating your constants from your main code is helpful because it makes it much easier to find values to tweak during tuning. Additionally, CopperCore (Team 401's core software library) provides JSONSync, a way to load constants with JSON files, which means you won't have to re-compile your code whenever you change a value. To see how to use JSON-Sync, see [how they're defined in 2025's ElevatorConstants.java](https://github.com/team401/2025-Robot-Code/blob/78548d16519e17e819c804a9da722a5ba1254a08/src/main/java/frc/robot/constants/subsystems/ElevatorConstants.java), [how they're loaded in JsonConstants.java](https://github.com/team401/2025-Robot-Code/blob/78548d16519e17e819c804a9da722a5ba1254a08/src/main/java/frc/robot/constants/JsonConstants.java), and [how they're accessed in 2025's WristIOTalonFX.java](https://github.com/team401/2025-Robot-Code/blob/78548d16519e17e819c804a9da722a5ba1254a08/src/main/java/frc/robot/subsystems/scoring/WristIOTalonFX.java#L77). This guide will access the constants as JSON constants, but you can just access them as class member variables if your team elects not to use JSON constants from CopperCore.

    ```java
    public ElevatorIOTalonFX() {
        // Initialize TalonFXs  and CANcoders with their correct IDs
        leadMotor = new TalonFX(JsonConstants.elevatorConstants.leadElevatorMotorId, "canivore");
        followerMotor =
            new TalonFX(JsonConstants.elevatorConstants.followerElevatorMotorId, "canivore");

        elevatorEncoder =
            new CANcoder(JsonConstants.elevatorConstants.elevatorCANCoderID, "canivore");

        CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
        cancoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
            JsonConstants.elevatorConstants.elevatorEncoderDiscontinuityPoint;

        cancoderConfiguration.MagnetSensor.SensorDirection =
            JsonConstants.elevatorConstants.elevatorElevatorEncoderDirection;
        cancoderConfiguration.MagnetSensor.MagnetOffset = 0.0;
        elevatorEncoder.getConfigurator().apply(cancoderConfiguration);

        // Initialize talonFXConfigs to use FusedCANCoder and Motion Magic Expo and have correct PID
        // gains and current limits.
        talonFXConfigs =
            new TalonFXConfiguration()
            // These configs dictate how Closed-Loop control will be run on the motors:
            .withFeedback(
                    new FeedbackConfigs()
                    // We set the elevator encoder as the "remote sensor". This means that the motor will listen to the current location of the elevator encoder as its position input for PID control
                    .withFeedbackRemoteSensorID(elevatorEncoder.getDeviceID())
                    // FusedCANCoder combines the encoder position and velocity with the readings from the internal encoder built into the motor; this provides faster and smoother control.
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    // Tell the motor how many rotations of the encoder are equivalent to one rotation of the "mechanism"
                    .withSensorToMechanismRatio(
                        JsonConstants.elevatorConstants.elevatorEncoderToMechanismRatio)
                    // This ratio tells the motor how many rotations of its internal rotor are equivalent to one rotation of the "elevator". This allows it to combine its inputs with the encoder's correctly with FusedCANCoder input.
                    .withRotorToSensorRatio(
                        JsonConstants.elevatorConstants.rotorToElevatorEncoderRatio))
            // This config means that the motor will spin freely when no power is requested; the other option is Brake, which will resist any movement, and can be useful in many cases as well.
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    // Current limits help to prevent overheating the motor and draining the battery.
                    .withStatorCurrentLimit(
                        JsonConstants.elevatorConstants.elevatorStatorCurrentLimit))
            .withSlot0(
                    // These Slot0Configs apply all of the gains for PID control as well as feedforward
                    new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKS(JsonConstants.elevatorConstants.elevatorkS)
                    .withKV(JsonConstants.elevatorConstants.elevatorkV)
                    .withKA(JsonConstants.elevatorConstants.elevatorkA)
                    .withKG(JsonConstants.elevatorConstants.elevatorkG)
                    .withKP(JsonConstants.elevatorConstants.elevatorkP)
                    .withKI(JsonConstants.elevatorConstants.elevatorkI)
                    .withKD(JsonConstants.elevatorConstants.elevatorkD))
            .withMotionMagic(
                    // This section of the config determines the maximum speed and acceleration, which MotionMagicExpo will use to generate a motion profile.
                    new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        JsonConstants.elevatorConstants.elevatorAngularCruiseVelocity)
                    .withMotionMagicExpo_kA(
                        JsonConstants.elevatorConstants.elevatorExpo_kA_raw)
                    .withMotionMagicExpo_kV(
                        JsonConstants.elevatorConstants.elevatorExpo_kV_raw));

        // Apply talonFX config to both motors
        leadMotor.getConfigurator().apply(talonFXConfigs);
        followerMotor.getConfigurator().apply(talonFXConfigs);

        // Make follower motor permanently follow lead motor.
        followerMotor.setControl(
                new Follower(
                    leadMotor.getDeviceID(),
                    JsonConstants.elevatorConstants.invertFollowerElevatorMotor));
    }
    ```
