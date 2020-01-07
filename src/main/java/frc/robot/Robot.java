package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cameraserver.CameraServer;
import java.lang.reflect.InvocationTargetException;
// import edu.wpi.first.cameraserver.*;
import com.zephyr.pixy.*;
import frc.robot.Toggle;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.SPILink;
//import sun.tools.jconsole.inspector.Utils;
import frc.robot.PairOfMotors;
import java.util.List;

// import javax.lang.model.util.ElementScanner6;

import java.util.ArrayList;


/*	

FIRST Robotics Team 1626
General Grievous
By Christopher Nokes and Christian Muce and SJHS Falcon Robotics

*/

public class Robot extends TimedRobot {

	private XboxController xbox;
	private Joystick driverLeft;
	private Joystick driverRight;

	private DriverStation driverStation;

	private SpeedController frontLeftSpeed;
	private SpeedController frontRightSpeed;
	private SpeedController backLeftSpeed;
	private SpeedController backRightSpeed;
	private SpeedControllerGroup leftSpeed;
	private SpeedControllerGroup rightSpeed;
	private DifferentialDrive drive;

	private WPI_TalonSRX ballHolder;
	private WPI_TalonSRX elevator;
	private WPI_TalonSRX leftArm;
	private WPI_TalonSRX rightArm;
	private WPI_TalonSRX frontJumper;
	private WPI_TalonSRX rearJumper;
	private SpeedControllerGroup jumperSpeed;

//	private TalonSRX inOutMotor1;
	private DoubleSolenoid claw;
	private DoubleSolenoid boost;
	int autoLoopCounter;
	public String gameData;
	private int startingPosition = 1;
	private ActionRecorder actions;
//	private Pixy pixycam;

	private Compressor compressor;
	private boolean compressorEnabled;
	private AnalogInput pressureSensor;

	private UsbCamera camera;
	private CameraServer cameraServer;
	private double previousElevator;

	private int ManualElevator = 0;
	private int RecordSpeed = 0;
	private int AxisRecord = 0;

	private double  SpeedVariable = 1;

	private List<PairOfMotors> motorPairList;

	private boolean hasPixy = true;
	private Pixy2CCC tracker;
	private Pixy2 pixy;

	private int liftRecord;

	Toggle backwards;
	Toggle doMotorBreakIn = new Toggle();
	Toggle clawState;
	Toggle boostState;
	Toggle dpadState;
	Toggle ballState;

	private int normArmCurrent=15;
	private int maxArmCurrent=35;
	private int armCurrent=normArmCurrent;

	private final int jumperOutTarget = 3300;
	private final int jumperInTarget = 0;


	@Override
	public void robotInit() {

		System.out.println("Starting the Deep Space Robot");

		cameraServer = CameraServer.getInstance();
		camera = new UsbCamera("USB Camera 0", 1);
		cameraServer.addCamera(camera);
		cameraServer.startAutomaticCapture();

		driverStation = DriverStation.getInstance();

		driverLeft = new Joystick(0);
		driverRight = new Joystick(1);
		xbox = new XboxController(2);
		backwards = new Toggle();
		clawState = new Toggle();
		boostState = new Toggle();
		dpadState = new Toggle();
		ballState = new Toggle();

		System.out.println("initializing actions...");
		actions = new ActionRecorder().
				setMethod(this, "robotOperation", DriverInput.class).
				setUpButton(xbox, 1).
				setDownButton(xbox, 2).
				setRecordButton(xbox, 3);
		
		System.out.println("initializing buttons...");
		DriverInput.nameInput("Driver-Left");
		DriverInput.nameInput("Driver-Right");
		DriverInput.nameInput("Driver-Left-Trigger");
		DriverInput.nameInput("Driver-Right-Trigger");
		DriverInput.nameInput("Operator-Left-Stick");
		DriverInput.nameInput("Operator-Left-Bumper");
		DriverInput.nameInput("Operator-Left-Trigger");
		DriverInput.nameInput("Operator-Right-Stick");
		DriverInput.nameInput("Operator-Right-Bumper");
		DriverInput.nameInput("Operator-Right-Trigger");
		DriverInput.nameInput("Operator-X-Button");
		DriverInput.nameInput("Operator-Y-Button");
		DriverInput.nameInput("Operator-A-Button");
		DriverInput.nameInput("Operator-B-Button");
		DriverInput.nameInput("Operator-Start-Button");
		DriverInput.nameInput("Operator-Back-Button");
		DriverInput.nameInput("Elevator-Forward");
		DriverInput.nameInput("Elevator-Back");
		DriverInput.nameInput("Operator-DPad");
		DriverInput.nameInput("Driver-Left-8");
		DriverInput.nameInput("Operator-Right-Stick");

		frontLeftSpeed		= new CANSparkMax(14, MotorType.kBrushless);
		backLeftSpeed		= new CANSparkMax(15, MotorType.kBrushless);
		frontRightSpeed		= new CANSparkMax(20, MotorType.kBrushless);
		backRightSpeed		= new CANSparkMax(21, MotorType.kBrushless);

		leftArm				= new WPI_TalonSRX(2); 
		rightArm			= new WPI_TalonSRX(3);

		elevator			= new WPI_TalonSRX(6);

		ballHolder			= new WPI_TalonSRX(7);

		frontJumper			= new WPI_TalonSRX(12);
		rearJumper			= new WPI_TalonSRX(13);

		compressor = new Compressor();
		pressureSensor = new AnalogInput(0);

		ManualElevator = 0;
		
		claw = new DoubleSolenoid(2, 3);
		claw.set(Value.kForward);

		boost = new DoubleSolenoid(0, 1);
		boost.set(Value.kReverse);

		leftSpeed = new SpeedControllerGroup(frontLeftSpeed, backLeftSpeed);
		rightSpeed = new SpeedControllerGroup(frontRightSpeed, backRightSpeed);
		drive = new DifferentialDrive(leftSpeed, rightSpeed);

		jumperSpeed = new SpeedControllerGroup(frontJumper, rearJumper);

		ballHolder.setInverted(true);
//		frontElevator.follow(backElevator);
//		double value = 1; 
//		backElevator.configSetParameter(ParamEnum.eClearPositionOnQuadIdx, value, 0x00, 0x00, 10);
//		backElevator.configSetParameter(ParamEnum.eClearPositionOnLimitF, value, 0x00, 0x00, 10);
//		backElevator.configSetParameter(ParamEnum.eClearPositionOnLimitR, value, 0x00, 0x00, 10);= 


		motorPairList = new ArrayList<PairOfMotors>();

		motorPairList.add(new PairOfMotors("LeftDrive", frontLeftSpeed, 14, backLeftSpeed, 15));
		motorPairList.add(new PairOfMotors("RightDrive", frontRightSpeed, 0, backRightSpeed, 1));
		motorPairList.add(new PairOfMotors("ArmDrive", 2,3));
		motorPairList.add(new PairOfMotors("Climb", 12,13));

//		for (PairOfMotors motorPair : motorPairList) {
//            SmartDashboard.putString(
//            "Motors/" + motorPair.getName(), 
//            "No motor current differences detected");
//		}
	
		elevator.configNominalOutputForward(0, 30);
		elevator.configNominalOutputReverse(0, 30);
		elevator.configPeakOutputForward(1, 30);
		elevator.configPeakOutputReverse(-1, 30);
		elevator.configAllowableClosedloopError(0, 0, 30);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

		SensorCollection sc = new SensorCollection(elevator);

		elevator.config_kF(0, 0.1, 30);
		elevator.config_kP(0, 0.1, 30);
		elevator.config_kI(0, 0, 30);
		elevator.config_kD(0, 0, 30);
		elevator.setSensorPhase(false);
		sc.setQuadraturePosition(0, 30);
	}

	@Override
	public void robotPeriodic() {
		double pressure = (250.0 * (pressureSensor.getVoltage() / 5.0)) - 13;
		SmartDashboard.putString("DB/String 4", String.format("%.0f", pressure));

		CANSparkMax[] rightDriveControllers = {(CANSparkMax)frontRightSpeed, (CANSparkMax)backRightSpeed};

		StringBuffer str = new StringBuffer();
		for (CANSparkMax controller : rightDriveControllers) {
			double temp=controller.getMotorTemperature();
			int id=controller.getDeviceId();
			if (str.length() > 0) {
				str.append(", ");
			}
			str.append(String.format("%.0f(%d)", temp, id-20));
		}
		SmartDashboard.putString("DB/String 2", str.toString());

		CANSparkMax[] leftDriveControllers = {(CANSparkMax)frontLeftSpeed, (CANSparkMax)backLeftSpeed};

		str = new StringBuffer();

		for (CANSparkMax motor : leftDriveControllers) {
			double temp=motor.getMotorTemperature();
			int id=motor.getDeviceId();
			if (str.length() > 0) {
				str.append(", ");
			}
			str.append(String.format("%.0f(%d)", temp, id));
		}
		SmartDashboard.putString("DB/String 3", str.toString());
	}


	@Override
	public void autonomousInit() {
		
		autoLoopCounter = 0;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		actions.autonomousInit("LLL");

		ballState.setState(false);
  	}

	@Override
	public void autonomousPeriodic() {
	
		boolean doAutonomous = false;
		try{
			if ( doAutonomous && (actions != null) && actions.hasInputs() ) {
				actions.longPlayback(this, -1);
			}
			else {
				teleopPeriodic();
			}
		}catch (Exception e) { 
			System.out.println("AP: " + e.toString());
			e.printStackTrace(System.out);
		}

  	} 

	@Override
	public void teleopInit() {
		DriverInput.setRecordTime();
		actions.teleopInit();

		int jumpPosition = ((WPI_TalonSRX)frontJumper).getSelectedSensorPosition();
		System.out.println("Jumper position is: " + jumpPosition);

		ballState.setState(true);
	}

	@Override
	public void teleopPeriodic() {

		RobotStopWatch watch = new RobotStopWatch("teleopPeriodic");

		double matchTime = driverStation.getMatchTime();
		boolean opControl = driverStation.isOperatorControl();

//		SmartDashboard.putString("DB/String 2", String.format("%.1f %s", matchTime, opControl?"T":"A"));
		if (opControl && (matchTime < 30.0)) {
			if (armCurrent < maxArmCurrent) {
				rightArm.configContinuousCurrentLimit(maxArmCurrent, 30);
				leftArm.configContinuousCurrentLimit(maxArmCurrent, 30);
				armCurrent = maxArmCurrent;
			}
		}

		if (opControl && (matchTime < 20) && (matchTime > 0)) {
			if (compressorEnabled) {
				compressor.stop();
				compressorEnabled=false;
				System.out.println("Stopped Compressor at " + matchTime);
			}

		}


//		SmartDashboard.putString("DB/String 3", String.valueOf(armCurrent));
	
		try {
			actions.input(new DriverInput()
				.withInput("Operator-X-Button",		xbox.getXButton())
				.withInput("Operator-Y-Button",		xbox.getYButton())
				.withInput("Operator-A-Button", 	xbox.getAButton())
				.withInput("Operator-B-Button",		xbox.getBButton())
				.withInput("Operator-Start-Button",	xbox.getRawButton(8))
				.withInput("Operator-Back-Button",	xbox.getRawButton(7))
				.withInput("Elevator-Forward",  	xbox.getTriggerAxis(Hand.kLeft))
				.withInput("Elevator-Back",			xbox.getTriggerAxis(Hand.kRight))
				.withInput("Operator-DPad",			xbox.getPOV())
				.withInput("Driver-Left", 			driverLeft.getRawAxis(1))
				.withInput("Driver-Right", 			driverRight.getRawAxis(1))
				.withInput("Driver-Left-Trigger", 	driverLeft.getRawButton(1))
				.withInput("Driver-Right-Trigger", 	driverRight.getRawButton(1))
				.withInput("Operator-Left-Bumper",	xbox.getBumper(Hand.kLeft))
				.withInput("Operator-Right-Bumper", xbox.getBumper(Hand.kRight))
				.withInput("Driver-Left-8", 		driverLeft.getRawButton(8))
				.withInput("Driver-Left-7", 		driverLeft.getRawButton(7))
				.withInput("Driver-Left-6", 		driverLeft.getRawButton(6))
				.withInput("Operator-Left-Stick",	xbox.getY(Hand.kLeft)) 
				.withInput("Operator-Right-Stick", xbox.getRawButton(10))
			);	
		
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			e.printStackTrace();
		}

//		System.err.println(watch.toString());
		
		// pixycam.getAllDetectedObjects();
		
	}


	public void disabledInit() {
		actions.disabledInit();

		sparkDiagnostics((CANSparkMax) frontLeftSpeed);
		sparkDiagnostics((CANSparkMax) backLeftSpeed);


		sparkDiagnostics((CANSparkMax) frontRightSpeed);
		sparkDiagnostics((CANSparkMax) backRightSpeed);

		leftArm.configContinuousCurrentLimit(normArmCurrent, 30);
		rightArm.configContinuousCurrentLimit(normArmCurrent, 30);
		armCurrent=normArmCurrent;
		leftArm.enableCurrentLimit(true);
		rightArm.enableCurrentLimit(true);

		for (PairOfMotors pair : motorPairList) {
			pair.reset();
		}

		compressor.start();
		compressorEnabled=true;

		liftRecord=0;


	}

	public void disabledPeriodic() {
		actions.disabledPeriodic();


	}

	public void robotOperation(DriverInput input) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);
		
		if(input.getButton("Operator-Right-Bumper"))
		{
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
		}
	}

//	double gay = frontLeftSpeed.get();
	
//		System.err.println(watch.toString());
	

	public void sparkDiagnostics(CANSparkMax controller) {

		int canID = controller.getDeviceId();

		System.err.println("Checking faults in Spark " + canID);

		for (CANSparkMax.FaultID c : CANSparkMax.FaultID.values()) {
			if (controller.getFault(c)) {
				System.err.println("Spark " + canID + " " + c.toString() + " SET");
			}

			if (controller.getStickyFault(c)) {
				System.err.println("Spark " + canID + " " + c.toString() + " STICKY");
			}
		}
	}

}