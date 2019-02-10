/**
 * Name: Team6438AutonomousCraterSide
 * Purpose: This class contains instructions for autonomous
 *          Currently the actions (in order) are: Raise the linear slide to unlatch
 *          Sample Blocks, Drive to the Depot, and run to the crater - want to add fling the team marker.
 * Author: Bradley Abelman
 * Contributors: Matthew Batkiewicz, Matthew Kaboolian
 * Creation: 11/8/18
 * Last Edit: 1/2/19
 * Additional Notes: TO DO
 *                  Find linearCPI
 *                  Test encoder based driving
 *                  Integrate way to control linearActuator in OpMode
 *                  Distance from landing to gems: Approximately 34 inches
 *                  Height of bracket off the ground: 19 inches
 *
 **/
package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

//@Disabled    //Uncomment this if the op mode needs to not show up on the DS
@Autonomous(name = "Crater Side", group = "Team 6438 Autonomous")
public class Team6438CraterAutonomous extends LinearOpMode
{
    //First time evaluation
    private static boolean firstTime = true;

    //Reference to our hardware map
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    //Variables for TensorFlow
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Inits the hardware
        robot.init(hardwareMap);

        //Sets them to use encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Resets encoders
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Telemetry to let user know robot init
        telemetry.addData("Status: ", "Ready to Run");
        telemetry.update();

        //init the vuforia engine when the class is called forward (selected on DS)
        initVuforia();

        //Wait for the start button to be pressed by the driver
        waitForStart();

        //Boolean to ensure we only run the block check the first time around
        firstTime = true;

        //Sets the block int to 0 (default value)
        int block = 0;

        //While the program is running
        while (opModeIsActive())
        {
            if (firstTime)
            {
                //move the actuator up and over
                actuatorMove(1, 17800/robot.linearCPI);

                //Move forward to get a better view of the blocks
                encoderRobotDrive(1, 5, 5);

                //Query the tensorFlowEngine and set the block variable equal to the result
                block = queryTensorFlow();

                //Block logic (seperated into ifs because we need different motions depending on where the block is
                if (block == 1)
                {
                    //telemetery to show the user what path we're running
                    telemetry.addData("Path running currently: ", "center");
                    telemetry.update();
                    //sleep(500);
                    firstTime=false;

                    //Encoder movements to run over the block
                    encoderRobotDrive(.75, 47.875, 47.875);
                }
                else if (block == 2)
                {
                    //telemetery to show the user what path we're running
                    telemetry.addData("Path running currently: ", "right");
                    telemetry.update();
                    //sleep(500);
                    firstTime=false;

                    //Encoder movements to run over the block
                    encoderRobotDrive(.75, 25.5, 25.5);
                    encoderRobotDrive(.75, -17, 17);
                    encoderRobotDrive(.75, 27, 27);
                }
                else if (block == 3)
                {
                    //telemetery to show the user what path we're running
                    telemetry.addData("Path running currently: ", "left");
                    telemetry.update();
                    //sleep(500);
                    firstTime=false;
                    encoderRobotDrive(.75, -19.5, 19.5);
                    encoderRobotDrive(1, 24.5, 24.5);
                    encoderRobotDrive(.75, 18, -18);
                    encoderRobotDrive(1, 24, 24);
                }
            }

            telemetry.addData("Autonomous Complete", "True");
            telemetry.update();
        }
    }

    //Encoder drive method to drive the motors
    private void encoderRobotDrive(double speed, double leftInches, double rightInches)
    {
        //Declaring new targets
        int leftTarget, rightTarget;

        //Gets the motors starting positions
        int startLeftPosition = robot.leftMotor.getCurrentPosition();
        int startRightPosition = robot.rightMotor.getCurrentPosition();

        //Telemetry to show start position
        telemetry.addData("Left Start Position", startLeftPosition);
        telemetry.addData("Right Start Position", startRightPosition);

        //Ensure we are in op mode
        if (opModeIsActive())
        {
            //Using the current position and the new desired position send this to the motor
            leftTarget = startLeftPosition + (int) (leftInches * robot.CPI);
            rightTarget = startRightPosition + (int) (rightInches * robot.CPI);
            robot.leftMotor.setTargetPosition(leftTarget);
            robot.rightMotor.setTargetPosition(rightTarget);

            //Turns the motors to run to position mode
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the power to the absolute value of the speed of the method input
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            //While opMode is still active and the motors are going add telemetry to tell the user where its going
            while (opModeIsActive() && robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
            {
                telemetry.addData("Running to ", " %7d :%7d", leftTarget, rightTarget);
                telemetry.addData("Currently At", " %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            //When done stop all the motion and turn off run to position
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //Method to move the actuator
    private void actuatorMove(double speed, double inches)
    {
        //Set up a new target variable
        int newTarget;  //(x/robot.)

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newTarget = robot.linearActuator.getCurrentPosition() + (int) (inches * robot.linearCPI);

            //Passes this target
            robot.linearActuator.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.linearActuator.setPower(Math.abs(speed));

            // keep looping while we are still active, and the motor is running.
            while (opModeIsActive() && robot.linearActuator.isBusy())
            {
                // Display it for the driver.
                telemetry.addData("Actuator moving to", newTarget);
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

    //Method to move the actuator
    @Deprecated
    private void actuatorByTime(int time)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // reset the timeout time and start motion.
            robot.linearActuator.setPower(1);
            sleep(time);
            robot.linearActuator.setPower(0);
            //  sleep(250);   // optional pause after each move
        }
    }

    //Method to move the intake
    private void intakeMove(double speed, int position)
    {
        if(opModeIsActive()) {
            //Sets the target position
            robot.intakeMover.setTargetPosition(position);

            //tells the motor to run to postion
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

    //Method to move the intake spinner (probably not necessary in autonomous)
    private void intakeSpin(double speed, long duration)
    {
        robot.intakeSpinner.setPower(speed);
        telemetry.addData("Speed", speed);
        telemetry.update();
        sleep(duration);
    }

    //Method to extend the servo and allow it to drop the team marker.
    private void tossMarker()
    {
        //Sleep for a quater second to make sure the servo can perform the action
        sleep(250);

        //Toss the servo
        robot.teamMarkerServo.setPosition(robot.toss);

        //Sleep again to make sure markers is off
        sleep(250);
    }

    /**
     * Queries the tensorFlow engine to find the block
     * No parameter but returns a int to let program know where the block is
     * 1 = center
     * 2 = right
     * 3 = left
     *
     * IMPORTANT: ASSUMES RIGHT (SELECT WHICH ONE AND DELETE THIS)
     *
     * Notes: want to integrate confidence reading (done - set to variable in the hardware map class)
     *        max y values
     *        max timeout ( if timeout passes ends the code and defaults to center
     *        we are assuming left/right? (assuming means were checking the center and the unassumed side
     *        i.e. if we assume left were checking center and right so if center and right are silver minerals
     *        we know the gold is on the left
     **/
    private int queryTensorFlow()
    {
        while (opModeIsActive() )
        {
            //int to determine gold block location: 1 = center, 2 = right, 3 = left
            int goldLocation = 0;

            if (opModeIsActive())
            {
                //Call the init TFod method to get block detection ready
                initTfod();

                //Activate Tensor Flow Object Detection.
                if (robot.tfod != null)
                {
                    //Activates the tfod engine
                    robot.tfod.activate();

                    //Waits for 1/2 second to allow processor to catch up
                    sleep(500);
                }

                while (opModeIsActive())
                {
                    if (robot.tfod != null && firstTime == true)
                    {
                        // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
                        List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            for (Recognition recognition : updatedRecognitions)
                            {
                                telemetry.addData("imageWidth ", recognition.getImageWidth());
                                telemetry.addData("mineralLocation ", recognition.getLeft());
                                telemetry.update();

                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                                {
                                    telemetry.addData("Value", "Center");
                                    telemetry.addData("Confidence", recognition.getConfidence());
                                    telemetry.update();
                                    sleep(100);
                                    robot.tfod.shutdown();

                                    //block in the center
                                    return 1;
                                }
                                else
                                {
                                    telemetry.addData("Moving", "Right");
                                    telemetry.update();
                                    encoderRobotDrive(.75, 9.17, -9.17);
                                    sleep(100);

                                    List<Recognition> updatedRecognitions2 = robot.tfod.getUpdatedRecognitions();
                                    if (updatedRecognitions2 != null) {
                                        //noinspection LoopStatementThatDoesntLoop
                                        for (Recognition recognition2 : updatedRecognitions2)
                                        {
                                            if (recognition2.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                                telemetry.addData("value", "Right");
                                                telemetry.addData("Confidence", recognition.getConfidence());
                                                telemetry.update();
                                                robot.tfod.shutdown();

                                                //block on the right
                                                return 2;
                                            }
                                            else {
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
            if (robot.tfod != null)
            {
                robot.tfod.shutdown();
            }
        }
        return 0;
    }

    //Method to init the vuforia engine
    private void initVuforia()
    {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //Sets the vuforia key
        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;

        //Sets camera direction
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //Instantiate the Vuforia engine
        robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //Method to init the tfod engine
    private void initTfod()
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

    //End of class
}