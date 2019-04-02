/**
 * Name: Team6438AutonomousDepotSide
 * Purpose: This class contains instructions for autonomous
 *          Currently the actions (in order) are: Raise the linear slide to unlatch
 *          Sample Blocks, Drive to the Depot, and run to the crater - want to add fling the team marker.
 * Author: Bradley Abelman
 * Contributors: Matthew Batkiewicz, Matthew Kaboolian, David Stekol
 * Creation: 11/8/18
 * Last Edit: 1/2/19
 * Additional Notes: TO DO
 *                  Find linearCPI
 *                  Test encoder based driving
 *                  Integrate way to control linearActuator in OpMode
 *                  Distance from landing to gems: Approximately 34 inches
 *                  Height of bracket off the ground: 19 inches
 *
 *                  Do we want to double sample??????
 *                  How much time are we dealing with
 **/
package org.firstinspires.ftc.teamcode;

//Imports
import android.graphics.Color;
import java.util.Map;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

//@Disabled    //Uncomment this if the op mode needs to not show up on the DS
@Autonomous(name = "Blue Crater", group = "Team 6438 Autonomous")
public class BlueCraterAutonomous extends LinearOpMode {

    //Reference to our hardware map
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    //Reference to our coordinate object
    //private coordinateObject coordinateSystem = new coordinateObject(57.5, 86.5);

    //First time evaluation
    private static boolean firstTime = true;

    //booleans for movement
    private static boolean sleeps = true;           //we can potentially set this to false if we want to make autonomous faster
    private static double preciseSpeed = 0.6;      //Speeds for any turns/precise movements where accuracy is key
    private static double quickSpeed = 1;          //Speeds for any straight line/imprecise movements where speed is key
    private static double pauseDistance = 100;
    private static long pauseTime = 50;

    //Grid value variables for CURRENT Position
    //Starting values go as follows: Red Depot- 86.5, 57.5; Red Crater- 86.5, 86.5; Blue Crater- 57.5, 57.5; Blue Depot- 57.5, 86.5
    double mapX = 57.5, mapY = 86.5;

    // The IMU sensor object
    private BNO055IMU imu;

    //Distance sensor object
    private DistanceSensor sensorRange;
    private ColorSensor sensorColor;

    //Used for distance stops
    private boolean firstDetection = true;
    private double encoderRemainingDistance;

    // Used for gyro turns
    private Orientation currentAngle = new Orientation();
    private double turnTarget;
    private int direction;

    //Variables for TensorFlow
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    @Override
    public void runOpMode() throws InterruptedException {
        //Inits the hardware
        robot.init(hardwareMap);

        //Sets them to use encoders  //THIS MAY NOT BE NECESSARY
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Resets encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set 0 behavior to breaking
        robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set up the parameters with which we will use our IMU. Note that integration algorithm here just reports accelerations to the logcat log; it doesn't actually provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.temperatureUnit    = BNO055IMU.TempUnit.CELSIUS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Set up color/distance sensor
        sensorRange = robot.sensorDistance;
        sensorColor = robot.sensorColor;

        //Turn on LED for color/distance sensor
        sensorColor.enableLed(true);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Telemetry to let user know robot init
        telemetry.addData("Status: ", "Ready to Run");
        telemetry.addData("Map X: ", mapX);
        telemetry.addData("Map Y: ", mapY);
        telemetry.update();

        //init the vuforia engine when the class is called forward (selected on DS)
        initVuforia();

        //Makes sure the camera is looking at the center
        //robot.cameraMount.setPosition(robot.cameraMountCenter);

        //Wait for the start button to be pressed by the driver
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Boolean to ensure we only run the block check the first time around
        firstTime = true;

        //Sets the block int to 0 (default value)
        int block = 0;

        //While the program is running
        while (opModeIsActive()) {
            telemetry.addData("Temp ", imu.getTemperature());
            telemetry.update();

            //Move the actuator up and over
            pinionMove(1, 500);

            //Drive to tile corner
            encoderRobotDrive(1, 5);

            //Check Block
            //queryTensorFlow();
            block = 1;

            //Block logic (separated into ifs because we need different motions depending on where the block is
            if (block == 1) {

                //Telemetry to show the user what path we're running
                telemetry.addData("Path running currently: ", "center");
                telemetry.update();

                //Movement to hit the block and return
                encoderRobotDrive(1, 15);
                encoderRobotDrive(1, -15);
                //sleep(500);
                firstTime = false;
            }
            else if (block == 2) {
                //Telemetry to show the user what path we're running
                telemetry.addData("Path running currently: ", "right");
                telemetry.update();

                //Movement to hit the block and return
                encoderRobotStrafe(1, 40);
                encoderRobotDrive(1, 15);
                encoderRobotDrive(1, -15);
                encoderRobotStrafe(1, -20);
                //sleep(500);
                firstTime = false;
            }
            else if (block == 3) {
                //Telemetry to show the user what path we're running
                telemetry.addData("Path running currently: ", "left");
                telemetry.update();

                //Movement to hit the block and return
                encoderRobotDrive(1, 15);
                encoderRobotDrive(1, -15);
                encoderRobotStrafe(1, 20);
                //sleep(500);
                firstTime = false;
            }

            //Turn and towards wall
            gyroRobotTurn(45);
            encoderRobotStrafe(1, 10);

            //Move to depot, extend intake and drop marker
            encoderRobotDrive(1, -50);
            tossMarker(1500);

            //Move to crater and extend arm
            encoderRobotDrive(1, 40);
            intakeRotate(.4, 1700);
            intakeExtend(1);

            //Lets the user know the Autonomous is complete
            telemetry.addData("Autonomous Complete", "True");
            telemetry.update();

            //End the opMode
            requestOpModeStop();

            //add diagnostic telemetry, this should never be shown
            telemetry.addData("If you see this: ", "it's too late");
            telemetry.update();
        }
    }

    /**
     * Encoder drive method to drive the motors, adds telemetry while the motors are busy
     *
     * @param: Speed, inches for left and right
     */

    //Hardware methods
        //Method to move the robot straight
        private void encoderRobotDrive ( double speed, double inches)
        {
            //Declaring new targets
            int target;
            int remainingDistance;

            //Gets the motors starting positions
            int startFLPosition = robot.leftFrontMotor.getCurrentPosition();
            int startFRPosition = robot.rightFrontMotor.getCurrentPosition();
            int startRLPosition = robot.leftRearMotor.getCurrentPosition();
            int startRRPosition = robot.rightRearMotor.getCurrentPosition();


            //Telemetry to show start position
            telemetry.addData("Front Left Start Position", startFLPosition);
            telemetry.addData("Front Right Start Position", startFRPosition);
            telemetry.addData("Rear Left Start Position", startRLPosition);
            telemetry.addData("Rear Right Start Position", startRRPosition);

            //Ensure we are in op mode
            if (opModeIsActive()) {
                //Using the current position and the new desired position send this to the motor
                target = startFLPosition + (int) (inches * robot.hexCPI);
                robot.leftFrontMotor.setTargetPosition(target);
                robot.rightFrontMotor.setTargetPosition(target);
                robot.leftRearMotor.setTargetPosition(target);
                robot.rightRearMotor.setTargetPosition(target);

                //Turns the motors to run to position mode
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Sets the power to the absolute value of the speed of the method input
                robot.leftFrontMotor.setPower(Math.abs(speed));
                robot.rightFrontMotor.setPower(Math.abs(speed));
                robot.leftRearMotor.setPower(Math.abs(speed));
                robot.rightRearMotor.setPower(Math.abs(speed));

                //While opMode is still active and the motors are going add telemetry to tell the user where its going
                while (opModeIsActive() && robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftRearMotor.isBusy() && robot.rightRearMotor.isBusy()) {
                    remainingDistance = target - robot.leftFrontMotor.getCurrentPosition();
                    telemetry.addData("Remaining Distance: ", remainingDistance);
                    telemetry.addData("Currently At", robot.leftFrontMotor.getCurrentPosition());
                    telemetry.update();

                    while (sensorRange.getDistance(DistanceUnit.CM) <= pauseDistance) {
                        robot.leftFrontMotor.setPower(0);
                        robot.rightFrontMotor.setPower(0);
                        robot.leftRearMotor.setPower(0);
                        robot.rightRearMotor.setPower(0);
                        sleep(pauseTime);
                        pauseAutonomous(pauseTime, remainingDistance, remainingDistance, remainingDistance, remainingDistance,
                                speed, speed, speed, speed);
                    }
                }
                //When done stop all the motion and turn off run to position
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftRearMotor.setPower(0);
                robot.rightRearMotor.setPower(0);
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        //Method to strafe the robot (Positive inches is right, negative inches is left)
        private void encoderRobotStrafe ( double speed, double inches)
        {
            //Declaring new targets
            int fLTarget, fRTarget, rLTarget, rRTarget;
            int fLRemainingDistance, fRRemainingDistance;

            //Gets the motors starting positions
            int startFLPosition = robot.leftFrontMotor.getCurrentPosition();
            int startFRPosition = robot.rightFrontMotor.getCurrentPosition();
            int startRLPosition = robot.leftRearMotor.getCurrentPosition();
            int startRRPosition = robot.rightRearMotor.getCurrentPosition();

            //Ensure we are in op mode
            if (opModeIsActive()) {
                //Using the current position and the new desired position send this to the motor
                fLTarget = startFLPosition - (int) (inches * robot.hexCPI);
                fRTarget = startFRPosition + (int) (inches * robot.hexCPI);
                rLTarget = startRLPosition + (int) (inches * robot.hexCPI);
                rRTarget = startRRPosition - (int) (inches * robot.hexCPI);
                robot.leftFrontMotor.setTargetPosition(fLTarget);
                robot.rightFrontMotor.setTargetPosition(fRTarget);
                robot.leftRearMotor.setTargetPosition(rLTarget);
                robot.rightRearMotor.setTargetPosition(rRTarget);

                //Turns the motors to run to position mode
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Sets the power to the absolute value of the speed of the method input
                robot.leftFrontMotor.setPower(Math.abs(speed));
                robot.rightFrontMotor.setPower(Math.abs(speed));
                robot.leftRearMotor.setPower(Math.abs(speed));
                robot.rightRearMotor.setPower(Math.abs(speed));

                //While opMode is still active and the motors are going add telemetry to tell the user where its going
                while (opModeIsActive() && robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftRearMotor.isBusy() && robot.rightRearMotor.isBusy()) {
                    fLRemainingDistance = fLTarget - robot.leftFrontMotor.getCurrentPosition();
                    fRRemainingDistance = fRTarget - robot.rightFrontMotor.getCurrentPosition();
                    telemetry.addData("Running to ", fRTarget);
                    telemetry.addData("Currently At", robot.leftFrontMotor.getCurrentPosition());
                    telemetry.update();

                    while (sensorRange.getDistance(DistanceUnit.CM) <= pauseDistance) {
                        pauseAutonomous(pauseTime, fLRemainingDistance, fRRemainingDistance, fLRemainingDistance, fRRemainingDistance,
                                speed, speed, speed, speed);
                    }
                }

                //When done stop all the motion and turn off run to position
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftRearMotor.setPower(0);
                robot.rightRearMotor.setPower(0);
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        //Method to extend the intake
        private void intakeExtend ( double speed)
        {
            //Set the target
            robot.intakeSlide.setTargetPosition(robot.slideExtended);

            //Turn On RUN_TO_POSITION
            robot.intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Start motion
            robot.intakeSlide.setPower(speed);

            while (robot.intakeSlide.isBusy()) {
                telemetry.addData("Moving to", robot.intakeSlide.getTargetPosition());
                telemetry.addData("Currently At", robot.intakeSlide.getCurrentPosition());
                telemetry.update();
            }
        }
        //Method to rotate the intake
        private void intakeRotate ( double speed, int position)
        {
            if (opModeIsActive()) {
                //Sets the target position
                robot.intakeMover.setTargetPosition(position);

                //tells the motor to run to position
                robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //sets the power
                robot.intakeMover.setPower(speed);

                //while moving is in progress telemetry for user
                while (robot.intakeMover.isBusy() && opModeIsActive()) {
                    telemetry.addData("Going to: ", position);
                    telemetry.addData("Currently At: ", robot.intakeMover.getCurrentPosition());
                    telemetry.update();
                }
            }
        }
        //Method to move the rack and pinion
        private void pinionMove ( double speed, int position)
        {
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                //Passes this target
                robot.linearActuator.setTargetPosition(position);

                // Turn On RUN_TO_POSITION
                robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                robot.linearActuator.setPower(Math.abs(speed));

                // keep looping while we are still active, and the motor is running.
                while (opModeIsActive() && robot.linearActuator.isBusy()) {
                    // Display it for the driver.
                    telemetry.addData("Actuator moving to", robot.linearActuator.getTargetPosition());
                    telemetry.addData("Actuator at", robot.linearActuator.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.linearActuator.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
        //Method to extend the servo and allow it to drop the team marker.
        private void tossMarker ( long pause)
        {
            //Toss the servo
            robot.leftIntake.setPower(robot.toss);
            robot.rightIntake.setPower(robot.toss);

            //Sleep to run the servos for a set amount of time
            sleep(pause);

            //Stop spinning the intake
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

    //Movement pause method
        //Method to stop the autonomous if a robot is detected by the distance sensor
        private void pauseAutonomous ( long time,
                                       int encoderRemainingDistanceFL, int encoderRemainingDistanceFR,
                                       int encoderRemainingDistanceBL, int encoderRemainingDistanceBR,
                                       double motorPowerOriginalFL, double motorPowerOriginalFR,
                                       double motorPowerOriginalBL, double motorPowerOriginalBR)
        {
        while (opModeIsActive()) {
            if (firstDetection) {
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.leftFrontMotor.setPower(-1);
                robot.rightFrontMotor.setPower(-1);
                robot.leftRearMotor.setPower(-1);
                robot.rightRearMotor.setPower(-1);
                sleep(time);
                firstDetection = false;

                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftRearMotor.setPower(0);
            robot.rightRearMotor.setPower(0);
            sleep(time);

            if (sensorRange.getDistance(DistanceUnit.CM) <= pauseDistance) {
                pauseAutonomous(pauseTime, encoderRemainingDistanceFL, encoderRemainingDistanceFR, encoderRemainingDistanceBL, encoderRemainingDistanceBR,
                        motorPowerOriginalFL, motorPowerOriginalFR, motorPowerOriginalBL, motorPowerOriginalBR);
            }
            robot.leftFrontMotor.setTargetPosition(encoderRemainingDistanceFL);
            robot.rightFrontMotor.setTargetPosition(encoderRemainingDistanceFR);
            robot.leftRearMotor.setTargetPosition(encoderRemainingDistanceBL);
            robot.rightRearMotor.setTargetPosition(encoderRemainingDistanceBR);

            robot.leftFrontMotor.setPower(motorPowerOriginalFL);
            robot.rightFrontMotor.setPower(motorPowerOriginalFR);
            robot.leftRearMotor.setPower(motorPowerOriginalBL);
            robot.rightRearMotor.setPower(motorPowerOriginalBR);
            firstDetection = true;

        }

    }

    //Gyro methods
        //Get facing angle
        private void getHeading ()
        {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        //Math to determine what angle to turn to
        private double getTarget ( double degrees)
        {

        double newAngle;
        turnTarget = degrees;

        if ((currentAngle.firstAngle + turnTarget) > 180) {
            newAngle = currentAngle.firstAngle + turnTarget - 360;
        } else if ((currentAngle.firstAngle + turnTarget) < -180) {
            newAngle = currentAngle.firstAngle + turnTarget + 360;
        } else {
            newAngle = currentAngle.firstAngle + turnTarget;
        }
        return newAngle;
    }
        //Turn the robot based on gyro readings
        /**
         * @condition degrees must be between -180 and 180 "https://stemrobotics.cs.pdx.edu/node/7265"
         * method to turn the robot a certain amount of degrees using the gyro
         * @param: degrees as a double
         **/
        private void gyroRobotTurn ( double degrees)
        {
            getHeading();

            //Ensure we are in op mode
            if (opModeIsActive()) {
                double speed = 0.4;
                turnTarget = getTarget(degrees);

                if (degrees < 0) {
                    direction = 1;
                } else {
                    direction = -1;
                }
                //Run by power
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.leftFrontMotor.setPower(speed * direction);
                robot.rightFrontMotor.setPower(speed * -direction);
                robot.leftRearMotor.setPower(speed * direction);
                robot.rightRearMotor.setPower(speed * -direction);


                while (currentAngle.firstAngle < turnTarget - 1 || currentAngle.firstAngle > turnTarget + 1) {
                    telemetry.addData("Speed", speed);
                    telemetry.addData("Running to ", turnTarget);
                    telemetry.addData("Currently At ", currentAngle.firstAngle);
                    telemetry.update();
                    getHeading();
                }

                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftRearMotor.setPower(0);
                robot.rightRearMotor.setPower(0);

                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Resets encoders
                robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

    //Map methods
        //Tells the robot to turn to and move to a point on the grid map
        private void mapAngleMove ( double speed, double x, double y )
        {
            double newAngle, distance;
            distance = Math.sqrt((y - mapY) * (y - mapY) + (x - mapX) * (x - mapX));

            if (y > mapY) {
                newAngle = -Math.toDegrees(Math.tan((y - mapY) / (x - mapX)));
            } else {
                newAngle = Math.toDegrees(Math.tan((y - mapY) / (x - mapX)));
            }

            while (opModeIsActive()) {
                getHeading();
                if (newAngle != currentAngle.firstAngle) {
                    gyroRobotTurn(newAngle);
                }
                encoderRobotDrive(speed, distance);
                mapX = x - mapX;
                mapY = y - mapY;

            }
            telemetry.addData("Map X: ", mapX);
            telemetry.addData("Map Y: ", mapY);
            telemetry.update();
        }
        //Tells the robot to strafe and drive straight to a point on the grid map
        private void mapStraightMove ( double speed, double x, double y )
        {
            double targetX, targetY;
            targetX = x - mapX;
            targetY = y - mapY;

            while (opModeIsActive()) {
                encoderRobotDrive(speed, targetY);
                encoderRobotStrafe(speed, targetX);
                mapX = targetX;
                mapY = targetY;
            }
            telemetry.addData("Map X: ", mapX);
            telemetry.addData("Map Y: ", mapY);
            telemetry.update();
        }

    //Vuforia and TFOD methods
        //Method to init the vuforia engine
        private void initVuforia ()
        {
            //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            //Sets the vuforia key
            parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;

            //Sets camera direction
            //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //Instantiate the Vuforia engine
            robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }
        //Method to init the tfod engine
        private void initTfod ()
        {
            //creates a tfod object which is using the tfodMonitor
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            //creates a new parameters object
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

            //Sets the minimum confidence for the program to read a block to the values stored in the Team6438HardwareMap
            tfodParameters.minimumConfidence = robot.confidence;

            //sets the tfod object equal to a new tfod object with parameters
            robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, robot.vuforia);

            //Loads this years assets
            robot.tfod.loadModelFromAsset(robot.TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, robot.LABEL_SILVER_MINERAL);
        }
        //Queries the tensorFlow engine to find the block
        /**
         *
         * No parameter but returns a int to let program know where the block is
         * 1 = center
         * 2 = right
         * 3 = left
         *
         * IMPORTANT: ASSUMES LEFT
         *
         * Notes: want to integrate confidence reading (done - set to variable in the hardware map class)
         *        max y values
         *        max timeout ( if timeout passes ends the code and defaults to center
         *        we are assuming left/right? (assuming means were checking the center and the unassumed side
         *        i.e. if we assume left were checking center and right so if center and right are silver minerals
         *        we know the gold is on the left
         **/
        private int queryTensorFlow ()
        {
            while (opModeIsActive()) {
                if (opModeIsActive()) {
                    //Call the init TFod method to get block detection ready
                    initTfod();

                    //Activate Tensor Flow Object Detection.
                    if (robot.tfod != null) {
                        //Activates the tfod engine
                        robot.tfod.activate();

                        //Waits for 1/2 second to allow processor to catch up
                        if (sleeps) sleep(500);
                    }

                    while (opModeIsActive()) {
                        if (robot.tfod != null && firstTime) {
                            sleep(100);
                            // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
                            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                for (Recognition recognition : updatedRecognitions) {
                                /*
                                telemetry.addData("imageWidth ", recognition.getImageWidth());
                                telemetry.addData("mineralLeft ", recognition.getLeft());
                                telemetry.addData( "mineralRight ", recognition.getRight());
                                telemetry.addData( "mineralNumber", updatedRecognitions.size());
                                telemetry.update();
                                */
                                    if (recognition.getRight() > 200 && recognition.getLeft() < 500) {
                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                            telemetry.addData("Value", "Center");
                                            telemetry.addData("Confidence", recognition.getConfidence());
                                            telemetry.update();
                                            sleep(100);
                                            robot.tfod.shutdown();

                                            //block in the center
                                            return 1;
                                        } else {
                                            encoderRobotStrafe(1, -20);
                                            telemetry.addData("Scanning Right", "Right");
                                            telemetry.update();
                                            sleep(100);

                                            List<Recognition> updatedRecognitions2 = robot.tfod.getUpdatedRecognitions();
                                            if (updatedRecognitions2 != null) {
                                                //noinspection LoopStatementThatDoesntLoop
                                                for (Recognition recognition2 : updatedRecognitions2) {

                                                    telemetry.addData("imageWidth2 ", recognition2.getImageWidth());
                                                    telemetry.addData("mineralLeft2 ", recognition2.getLeft());
                                                    telemetry.addData("mineralRight2 ", recognition2.getRight());
                                                    telemetry.update();

                                                    if (recognition2.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                                        telemetry.addData("value", "Right");
                                                        telemetry.addData("Confidence", recognition.getConfidence());
                                                        telemetry.update();
                                                        robot.tfod.shutdown();

                                                        //block on the right
                                                        return 2;
                                                    } else {
                                                        telemetry.addData("value", "Left");
                                                        telemetry.addData("Confidence", recognition.getConfidence());
                                                        telemetry.update();
                                                        robot.tfod.shutdown();

                                                        //block on the left
                                                        return 3;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if (robot.tfod != null) {
                    robot.tfod.shutdown();
                }
            }
            return 0;
        }

    //End of class
}