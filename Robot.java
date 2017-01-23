package org.usfirst.frc.team6305.robot;
import org.usfirst.frc.team6305.robot.Pipeline;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String baseLine = "Base Line";
	final String highGoal = "High Goal";
	final String gear = "Gear";
	final String hgHopper = "High Goal w/ Hopper";
	
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	
	private final Object imgLock = new Object();
	
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	//NI keycode: m28x13666
	
	//Define motors
	Spark frontLeft = new Spark(0);
    Spark rearLeft = new Spark (1);
    Spark frontRight = new Spark(2);
    Spark rearRight = new Spark(3);
    Spark intakeFly = new Spark(4);
    Spark intakeEl = new Spark(5);
    Spark outputFly = new Spark(6);
    
    
	//Initialize joysticks
	Joystick jRstick = new Joystick(0);
	Joystick jLstick = new Joystick(1);
	XboxController xStick = new XboxController(2);
	
	private RobotDrive myDrive;
	private Gyro myGyro;
	
	double Kp = 0.03;
	
	//Start match timer
	Timer timer = new Timer();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 * @return 
	 */
	
	public void GyroMethod(){
		myGyro = new AnalogGyro(1);
		myDrive = new RobotDrive (frontLeft, rearLeft, frontRight, rearRight);
		myDrive.setExpiration(0.1);
	}
	@Override
	public void robotInit() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    VisionThread targeting = new VisionThread(camera, new Pipeline());
	        if (!targeting.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(targeting.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	            }
	        }
	    
	    visionThread.start();
	    
		//auto cases here 
		chooser.addDefault("Stationary", defaultAuto);
		chooser.addObject("Base Line", baseLine);
		chooser.addObject("Gear", gear);
		chooser.addObject("High Goal", highGoal);
		chooser.addObject("High Goal w/ Hopper", hgHopper);
		SmartDashboard.putData("Auto choices", chooser);
		}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		//autoSelected = SmartDashboard.getString("Auto Selector",
		//defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		timer.reset();
		timer.start(); 
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	//Declare RobotDrive seperately for auto and teleop to fix for forward direction (make sure to declare joysticks in the right order)
	@Override
	public void autonomousPeriodic() {
		//RobotDrive autoDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		double turn = centerX - (IMG_WIDTH / 2);
		myDrive.drive(-0.6, turn * 0.005);
		myGyro.reset();
		while(isAutonomous() && isEnabled()){
			double angle = myGyro.getAngle();
			myDrive.drive(-1.0, -angle*Kp);
			Timer.delay(0.004);
			switch (autoSelected) {
			
			case defaultAuto:
				// nothing
				break;
			 
			case baseLine:
				if(timer.get()<4){
					//THIS SHOULD GO STRAIGHT
					//curve= .(12 zeros)1
					//minimal curve counteracts curve (due to motors going @ diff speeds?) 
					myDrive.drive(.5,.0000000000001);
					
				} else {
					myDrive.drive(0, 0);
				}
					break;
					
			case gear:
				while(timer.get()<1){
				myDrive.drive(.3,0);
				}
				while(timer.get()>1 && timer.get()<2){
					myDrive.drive(-.3,0);
					}
				break;
				
			case hgHopper: 
				break;
				
			case highGoal:
				while(timer.get()<3){
					myDrive.drive(.5, 0);
				} 
				Timer.delay(1);
				while(timer.get()>3 && timer.get()<4){
				myDrive.drive(.5, -.4);
				} while(timer.get()<15){
				//vis software search loop until statement is true 
					//then shoot 
				}

				break;
	
				
			}
						
			
			break;
		}
	}


	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//RobotDrive teleopDrive = new RobotDrive(rearRight, frontRight, rearLeft, frontLeft);
		while (isOperatorControl() && isEnabled()) {
		myDrive.tankDrive(jLstick, jRstick);
		Timer.delay(0.005);
		//Intake -- right bumper
		if(xStick.getBumper(GenericHID.Hand.kRight)){
			intakeFly.set(.5);
			}
		//Reverse Intake (if clogged) -- left bumper
		if(xStick.getBumper(GenericHID.Hand.kLeft)){
			intakeFly.set(-.5);
			}
		//Auto-targeting -- a
		if(xStick.getAButton());
			
		if(xStick.getTrigger(GenericHID.Hand.kRight)){
			//set motor for output for low goal
			}
		
		if(xStick.getTrigger(GenericHID.Hand.kLeft)){
			outputFly.set(.5);
			
			}
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

