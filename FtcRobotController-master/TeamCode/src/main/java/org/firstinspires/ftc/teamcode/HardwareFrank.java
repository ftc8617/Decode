package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

/*
 * Hardware class for 2024-2025 IntoTheDeep V2 robot
 */
public class HardwareFrank
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;
    public double controlHubV   = 0.0; // Voltage supply of the control hub
    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected BNO055IMU imu    = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;
    //protected GoBildaPinpointDriver odom = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          frontLeftMotorPos  = 0;       // current encoder count
    public double       frontLeftMotorVel  = 0.0;     // encoder counts per second
    public double       frontLeftMotorAmps = 0.0;     // current power draw (Amps)

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorTgt = 0;       // RUN_TO_POSITION target encoder count
    public int          frontRightMotorPos = 0;       // current encoder count
    public double       frontRightMotorVel = 0.0;     // encoder counts per second
    public double       frontRightMotorAmps= 0.0;     // current power draw (Amps)

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorTgt   = 0;       // RUN_TO_POSITION target encoder count
    public int          rearLeftMotorPos   = 0;       // current encoder count
    public double       rearLeftMotorVel   = 0.0;     // encoder counts per second
    public double       rearLeftMotorAmps  = 0.0;     // current power draw (Amps)

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          rearRightMotorPos  = 0;       // current encoder count
    public double       rearRightMotorVel  = 0.0;     // encoder counts per second
    public double       rearRightMotorAmps = 0.0;     // current power draw (Amps)

    public final static double MIN_DRIVE_POW      = 0.03;    // Minimum speed to move the robot
    public final static double MIN_TURN_POW       = 0.03;    // Minimum speed to turn the robot
    public final static double MIN_STRAFE_POW     = 0.04;    // Minimum speed to strafe the robot
    protected double COUNTS_PER_MOTOR_REV  = 28.0;    // goBilda Yellow Jacket Planetary Gear Motor Encoders
//  protected double DRIVE_GEAR_REDUCTION  = 26.851;  // goBilda 26.9:1 (223rpm) gear ratio with 1:1 HDT5 pully/belt
    protected double DRIVE_GEAR_REDUCTION  = 19.203;  // goBilda 19.2:1 (312rpm) gear ratio with 1:1 HDT5 pully/belt
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 3.77953; // (96mm) -- for computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // The math above assumes motor encoders.  For REV odometry pods, the counts per inch is different
    protected double COUNTS_PER_INCH2      = 1738.4;  // 8192 counts-per-rev / (1.5" omni wheel * PI)



    public int positionLError;
    public int positionRError;

    protected DcMotorEx pigChucker     = null;
    public int          pigChuckerTarget  = 0;       // TODO: fill out hub positions
    public int pigChuckerPos  = 0;      //
    public double       pigChuckerVelocity  = 0.0;     //

    protected DcMotorEx intakeMotor     = null;
    public int          intakeMotorPos  = 0;       //
    public double       intakeMotorVelocity  = 0.0;     //

    public Servo turntableServo   = null;
    public Servo flipperServo = null;

    public double  turntablePos   = 0;
    //public int turntableSlot = ;
    public double  flipperPos = 0;
    //public boolean flipperUp = false;

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareFrank(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous ) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Configure REV control/expansion hubs for bulk reads (faster!)
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
        // Locate the odometry controller in our hardware settings
        /*
        odom = hwMap.get(GoBildaPinpointDriver.class,"odom"); //Control Hub I2C port 3
        odom.setOffsets(111.23, 36.73);   // odometry pod x,y locations [mm] relative to center of robot
        odom.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD ); // 4bar pods
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                  GoBildaPinpointDriver.EncoderDirection.FORWARD);
        if( isAutonomous ) {
            odom.resetPosAndIMU();
        }
                 */

        // Define and Initialize drivetrain motors
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"leftFront");  // Expansion Hub port 1 (REVERSE)
        frontRightMotor = hwMap.get(DcMotorEx.class,"rightFront"); // Expansion Hub port 2 (forward)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"leftBack");   // Expansion Hub port 0 (REVERSE)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"rightBack");  // Expansion Hub port 3 (forward)

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();

        // Set all drivetrain motors to run with encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Define and initialize Swyft Slide motors
        intakeMotor= hwMap.get(DcMotorEx.class,"intakeMotor");
        pigChucker= hwMap.get(DcMotorEx.class,"pigChucker");


        intakeMotor.setPower( 0.0 );
        pigChucker.setPower( 0.0 );

        if(isAutonomous) {
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pigChucker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pigChucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize robot hardware (autonomous=true initializes servos)
        turntableServo   = hwMap.get(Servo.class,"turntable");            // servo port 0 (Expansion Hub)
        flipperServo = hwMap.get(Servo.class,"flipper");          // servo port 1 (Expansion Hub)

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    public void resetEncoders() { //TODO: Set init values
        turntablePos   = 0.855;     turntableServo.setPosition(turntablePos); //set init value
       flipperPos = 0.114;    flipperServo.setPosition(flipperPos);

       pigChucker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       pigChucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json"; // located in FIRST/settings folder
        imu_params.loggingEnabled = false;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize( imu_params );
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        headingAngle = angles.firstAngle;
        tiltAngle = angles.secondAngle;
        return -headingAngle;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        //===== CONTROL HUB VALUES =====
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        pigChuckerPos     = pigChucker.getCurrentPosition();
        intakeMotorPos     = intakeMotor.getCurrentPosition();
    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    // This is a slow operation (involves an I2C reading) so only do it as needed
    public double readBatteryControlHub() {
        // Update local variable and then return that value
        controlHubV = controlHub.getInputVoltage( VoltageUnit.MILLIVOLTS );
        return controlHubV;
    } // readBatteryControlHub



    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotors( double frontLeft, double frontRight, double rearLeft, double rearRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        rearLeftMotor.setPower( rearLeft );
        rearRightMotor.setPower( rearRight );
    } // driveTrainMotors

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to drive straight FORWARD (Ex: +0.10) or REVERSE (Ex: -0.10)        */
    public void driveTrainFwdRev( double motorPower )
    {
        frontLeftMotor.setPower(  motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(   motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainFwdRev

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to strafe RIGHT (Ex: +0.10) or LEFT (Ex: -0.10)                     */
    public void driveTrainRightLeft( double motorPower )
    {
        frontLeftMotor.setPower(   motorPower );
        frontRightMotor.setPower( -motorPower );
        rearLeftMotor.setPower(   -motorPower );
        rearRightMotor.setPower(   motorPower );
    } // driveTrainRightLeft

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to turn clockwise (Ex: +0.10) or counterclockwise (Ex: -0.10)       */
    public void driveTrainTurn( double motorPower )
    {
        frontLeftMotor.setPower( -motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(  -motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainTurn

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

    /*--------------------------------------------------------------------------------------------*/
    public void stopMotion() {
        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    /*--------------------------------------------------------------------------------------------*/

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    } /* waitForTick() */
} /* HardwareFrank */
