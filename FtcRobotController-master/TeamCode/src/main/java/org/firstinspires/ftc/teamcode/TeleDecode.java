/* FTC Team 8617 - Version 1.0 (10/01/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TeleOp (with test modes).
 */
@TeleOp(name="TeleDecode", group="8617")
public class TeleDecode extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;
    boolean gamepad1_square_last,     gamepad1_square_now     = false;
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  //
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  //
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  //
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  //
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  //
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  //
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  //
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  //
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  //
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  //
    boolean gamepad2_touchpad_last,   gamepad2_touchpad_now   = false;  //
    boolean gamepad2_share_last,      gamepad2_share_now      = false;  //

    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean controlMultSegLinear = true;

    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    double curX, curY, curAngle;
    double minX=0.0, maxX=0.0, minY=0.0, maxY=0.0;

    //rumble settings
    Gamepad.RumbleEffect shortRumble;
    Gamepad.RumbleEffect leftRumble;

    Gamepad.RumbleEffect leftDoubleRumble;
    Gamepad.RumbleEffect rightRumble;
    Gamepad.RumbleEffect rightDoubleRumble;

    /* Declare OpMode members. */
    HardwareFrank robot = new HardwareFrank();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        //creating rumble effects
        shortRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1,1,200) //rumble both sides for 200ms
                .build();

        leftRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1,0,200)
                .build();
        leftDoubleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1,0,200)
                .addStep(0,0,100)
                .addStep(1,0,200)
                .build();

        rightRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0,1,200)
                .build();
        rightDoubleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0,1,200)
                .addStep(0,0,100)
                .addStep(0,1,200)
                .build();

        // Initialize robot hardware (not autonomous mode)
        robot.init(hardwareMap,false);


        // Preload and initialize

        robot.pigChucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "austen james richardson");
        telemetry.addLine("Press X (cross) to reset encoders");
        telemetry.addLine("(to run Teleop without Auto first)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Normally autonomous resets encoders.  Do we need to for teleop??
            if( gamepad1_cross_now && !gamepad1_cross_last) {
                robot.resetEncoders();
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Refresh gamepad button status
            captureGamepad1Buttons();
            captureGamepad2Buttons();

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();



            if( processDpadDriveMode() == false ) {
                // Control based on joystick; report the sensed values
                telemetry.addData("Joystick", "x=%.3f, y=%.3f spin=%.3f",
                        gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x );
                processStandardDriveMode();
            } // processDpadDriveMode

            encoderReset();

            //add gameplay functions
            processIntake();
            processShooter();
            processTurntable();
            processKicker();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
            telemetry.addData("Front", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    frontLeft, robot.frontLeftMotorVel, frontRight, robot.frontRightMotorVel );
            telemetry.addData("Rear ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
            telemetry.addData("Front", "%d %d counts", robot.frontLeftMotorPos, robot.frontRightMotorPos );
            telemetry.addData("Back ", "%d %d counts", robot.rearLeftMotorPos,  robot.rearRightMotorPos );
            /* old telemetry, replace
//          telemetry.addData("Slide ", "%d %d counts", robot.slideLMotorPos,  robot.slideRMotorPos );
            telemetry.addData("Slide ", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
//          telemetry.addData("Chain ", "%d counts", robot.chainMotorPos );
            telemetry.addData("Chain ", "%d counts (%.1f)", robot.chainMotorPos, robot.chainMotor.getPower() );
            telemetry.addData("Claw ", "%.3f counts", robot.clawPos );
            telemetry.addData("Diffy ", "%.3f %.3f counts", robot.diffyLPos, robot.diffyRPos );
            */
            telemetry.addData("Kicker ", "%.3f counts", robot.flipperPos);
            telemetry.addData("Turntable ", "%.3f counts", robot.turntablePos);

            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.addData("version","103");
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
    } // captureGamepad1Buttons

    /*---------------------------------------------------------------------------------*/
    void captureGamepad2Buttons() {
        gamepad2_triangle_last   = gamepad2_triangle_now;    gamepad2_triangle_now   = gamepad2.triangle;
        gamepad2_circle_last     = gamepad2_circle_now;      gamepad2_circle_now     = gamepad2.circle;
        gamepad2_cross_last      = gamepad2_cross_now;       gamepad2_cross_now      = gamepad2.cross;
        gamepad2_square_last     = gamepad2_square_now;      gamepad2_square_now     = gamepad2.square;
        gamepad2_dpad_up_last    = gamepad2_dpad_up_now;     gamepad2_dpad_up_now    = gamepad2.dpad_up;
        gamepad2_dpad_down_last  = gamepad2_dpad_down_now;   gamepad2_dpad_down_now  = gamepad2.dpad_down;
        gamepad2_dpad_left_last  = gamepad2_dpad_left_now;   gamepad2_dpad_left_now  = gamepad2.dpad_left;
        gamepad2_dpad_right_last = gamepad2_dpad_right_now;  gamepad2_dpad_right_now = gamepad2.dpad_right;
        gamepad2_l_bumper_last   = gamepad2_l_bumper_now;    gamepad2_l_bumper_now   = gamepad2.left_bumper;
        gamepad2_r_bumper_last   = gamepad2_r_bumper_now;    gamepad2_r_bumper_now   = gamepad2.right_bumper;
//      gamepad2_touchpad_last   = gamepad2_touchpad_now;    gamepad2_touchpad_now   = gamepad2.touchpad;
//      gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.20;
        boolean dPadMode = false;
        // Only process 1 Dpad button at a time

        return dPadMode;
    } // processDpadDriveMode

    private double minThreshold( double valueIn ) {
        double valueOut;

        //========= NO/MINIMAL JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.02 ) {
            valueOut = 0.0;
        }
        else {
            valueOut = valueIn;
        }
        return valueOut;
    } // minThreshold

    private double multSegLinearRot( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                      // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0650;   // 0.02=0.070  0.33=0.1475
            }
            else if( valueIn < 0.60 ) {
                valueOut = (0.50 * valueIn) - 0.0175;   // 0.33=0.1475  0.60=0.2825
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.1675;   // 0.60=0.2825  0.90=0.5075
            }
            else
                valueOut = (6.00 * valueIn) - 4.8925;   // 0.90=0.5075  1.00=1.1075 (clipped!)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0650;
            }
            else if( valueIn > -0.60 ) {
                valueOut = (0.50 * valueIn) + 0.0175;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.1675;
            }
            else
                valueOut = (6.00 * valueIn) + 4.8925;
        }

        return valueOut/2.0;
    } // multSegLinearRot

    private double multSegLinearXY( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.50 ) {                       // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.040;     // 0.01=0.0425   0.50=0.1650
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.210;     // 0.50=0.1650   0.90=0.4650
            }
            else
                valueOut = (8.0 * valueIn) - 6.735;      // 0.90=0.4650   1.00=1.265 (clipped)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.50 ) {
                valueOut = (0.25 * valueIn) - 0.040;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.210;
            }
            else
                valueOut = (8.0 * valueIn) + 6.735;
        }

        return valueOut;
    } // multSegLinearXY

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Standard Mecanum-wheel drive control (no dependence on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processStandardDriveMode() {
        // new awesone roarbots code

        // Retrieve X/Y and ROTATION joystick input
        if( controlMultSegLinear ) {
            yTranslation = multSegLinearXY( .7 * -gamepad1.left_stick_y );
            xTranslation = multSegLinearXY(  .7 * gamepad1.left_stick_x );
            rotation     = multSegLinearRot( .6  * -gamepad1.right_stick_x );

        }
        else {
            yTranslation = -gamepad1.left_stick_y * .50;
            xTranslation =  gamepad1.left_stick_x * .50;
            rotation     = -gamepad1.right_stick_x * 0.65;
        }

        // Normal teleop drive control:
        // - left joystick is TRANSLATE fwd/back/left/right
        // - right joystick is ROTATE clockwise/counterclockwise
        // NOTE: assumes the right motors are defined FORWARD and the
        // left motors are defined REVERSE so positive power is FORWARD.
        frontRight = yTranslation - xTranslation + rotation;
        frontLeft  = yTranslation + xTranslation - rotation;
        rearRight  = yTranslation + xTranslation + rotation;
        rearLeft   = yTranslation - xTranslation - rotation;
        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                             Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        //auto break
        if ((gamepad1.left_stick_y < .15 && gamepad1.left_stick_y > -.15) && (gamepad1.left_stick_x < .15 && gamepad1.left_stick_x > -.15) && (gamepad1.right_stick_y < .15 && gamepad1.right_stick_y > -.15) && (gamepad1.right_stick_x < .15 && gamepad1.right_stick_x > -.15)) {
            frontLeft = 0;
            frontRight = 0;
            rearLeft = 0;
            rearRight = 0;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );


        /*
        //now for the legacy drive !
        //Finds the hypotenuse of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //Finds the robot's angle from the raw values of the joystick
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
            //reversed bc idek the old code just works this way
            frontLeft = (-0.75*v2);
            frontRight = (-0.75*v1);
            rearLeft = (-0.75*v4);
            rearRight = (-0.75*v3);
        }

        else if(gamepad1.left_bumper) {
            frontLeft = (-0.5);
            rearLeft = (-0.5);
            frontRight = (0.5);
            rearRight = (0.5);
        }

        else if(gamepad1.right_bumper) {
            frontLeft = (0.5);
            rearLeft = (0.5);
            frontRight = (-0.5);
            rearRight = (-0.5);
        }
        else if(gamepad1.right_trigger > 0.15) {
            double power = 0.25*gamepad1.right_trigger;
            frontLeft = (power);
            rearLeft = (power);
            frontRight = (-power);
            rearRight = (-power);
        }
        else if(gamepad1.left_trigger > 0.15) {
            double power = 0.25*gamepad1.left_trigger;
            frontLeft = (-power);
            rearLeft = (-power);
            frontRight = (power);
            rearRight = (power);
        }
        else {
            frontLeft = (0);
            rearLeft = (0);
            frontRight = (0);
            rearRight = (0);
        }
        old code
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
        */

    } // processStandardDriveMode
    void encoderReset (){
        if (gamepad2_square_now && !gamepad2_square_last) { // reset tele-op positions
            robot.pigChucker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.pigChucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);
            gamepad1.runRumbleEffect(shortRumble);
            gamepad2.runRumbleEffect(shortRumble);
        }
    } //encoderReset
    void processIntake(){
        if (gamepad1.right_trigger >= 0.03) {
            robot.intakeMotor.setPower(gamepad1.right_trigger ); //forward
        } else if (gamepad1.left_trigger >= 0.03) {
            robot.intakeMotor.setPower(-gamepad1.left_trigger ) ; //reverse
        } else {
            robot.intakeMotor.setPower(0);
        }
    } //processIntake

    void processShooter(){
        //gamepad 1 flywheel controls
        if (gamepad1_circle_now){
            robot.pigChucker.setPower(0.73);
            robot.pigSpinning = true;
        } else if (gamepad1_square_now){
            robot.pigChucker.setPower(0.63);
            robot.pigSpinning = true;
        } else if(gamepad1_cross_now){
            robot.pigChucker.setPower(0);
            robot.pigSpinning = false;
        }

        //gamepad 2 flywheel controlls
        if (gamepad2_circle_now){
            robot.pigChucker.setPower(0.73);
            robot.pigSpinning = true;
        } else if (gamepad2_square_now){
            robot.pigChucker.setPower(0.63);
            robot.pigSpinning = true;
        } else if(gamepad2_cross_now){
            robot.pigChucker.setPower(0);
            robot.pigSpinning = false;
        }

        //gamepad 2 rumble based on flywheel activation
        if (robot.pigSpinning){
            gamepad2.runRumbleEffect(shortRumble);
        }

    } //processShooter

    void processTurntable(){
        // we are going to need 6 positions for the turntable, and they can just run sequentially
        //one for intake and one for shooting thus they will be 1i,2i,3i, and then 1s,2s,3s
        /*
        if (gamepad1_triangle_now){
            robot.turntableSlot = 1;
        }
        if(gamepad1_r_bumper_now && !gamepad1_r_bumper_last){ //right bumper to intake
            if (robot.turntableSlot >= 3){
                robot.turntableSlot = 1;
            } else {
                robot.turntableSlot += 1;
            }
        } else if (gamepad1_l_bumper_now && !gamepad1_l_bumper_last){ //control shooting turntable with left bumper
            if (robot.turntableSlot <= 3) {
                robot.turntableSlot = 4;
            } else if (robot.turntableSlot <= 5) {
                robot.turntableSlot += 1;
            } else {
                robot.turntableSlot = 4;
            }
        }

        //process turntable pos
        //1 2 3 intake, 4 5 6 shooting
        //1i = 4o
        //2i = 5o
        //3i = 6o



        if(robot.turntableSlot == 1){ //TODO: fill out all turntable positions with accurate doubles
            robot.turntablePos = 0;
        } else if (robot.turntableSlot == 2){
            robot.turntablePos = 0;
        } else if (robot.turntableSlot == 3){
            robot.turntablePos = 0;
        } else if (robot.turntableSlot == 4){
            robot.turntablePos = 0;
        } else if (robot.turntableSlot == 5){
            robot.turntablePos = 0;
        } else if (robot.turntableSlot == 6){
            robot.turntablePos = 0;
        }
        */
        if (gamepad1_r_bumper_now){
            if (robot.turntablePos < 0.96){
                robot.turntablePos += 0.001;
            } else {
                gamepad1.runRumbleEffect(shortRumble);
            }
        } else if (gamepad1_l_bumper_now) {
            if (robot.turntablePos > 0.08) {
                robot.turntablePos -= 0.001;
            } else {
                gamepad1.runRumbleEffect(shortRumble);
            }
        }


        robot.turntableServo.setPosition(robot.turntablePos);
    } //processTurntable


    void processKicker(){
        if(gamepad1_dpad_down_now && !gamepad1_dpad_down_last){
            robot.flipperUp = !robot.flipperUp;
        }

        if(robot.flipperUp){
            robot.flipperPos = 0.600;
        } else if (!robot.flipperUp) {
            robot.flipperPos = 0.400;
        }

        robot.flipperServo.setPosition(robot.flipperPos);
    }

} // Teleop


