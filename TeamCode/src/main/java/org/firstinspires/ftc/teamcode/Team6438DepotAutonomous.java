/**
 * Name: Team6438AutonomousDepotSide
 * Purpose: This class contains instructions for autonomous
 *          Currently the actions (in order) are: Raise the linear slide to unlatch
 *          Sample Blocks, Drive to the Depot and fling the team marker.
 * Author: Bradley Abelman
 * Contributors: Matthew Batkiewicz, Matthew Kaboolian
 * Creation: 11/8/18
 * Last Edit: 1/2/19
 * Additional Notes: TO DO
 *                  Find linearCPI
 *                  Test encoder based driving
 *                  Integrate way to control linearSlide in OpMode
 *                  Distance from landing to gems: Approximately 34 inches
 *                  Height of bracket off the ground: 19 inches
 **/
package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//@Disabled    //Uncomment this if the op mode needs to not show up on the DS
@Autonomous(name = "Depot Side Autonomous", group = "Autonomous Team 6438")
public class Team6438DepotAutonomous extends LinearOpMode
{
    //Reference to our hardware map
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    //Variables for TensorFlow
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    //Boolean to ensure we only run the block check the first time around
    private static boolean firstTime = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Inits the hardware
        robot.init(hardwareMap);

        //Sets them to use encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Resets encoders
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Telemetry to let user know robot init
        telemetry.addData("Status: ", "Ready to Run");
        telemetry.update();

        //init the vuforia engine when the class is called forward (selected on DS)
        initVuforia();

        //Wait for the start button to be pressed by the driver
        waitForStart();

        //Sets the block int to 0 (default value)
        int block = 0;

        //While the program is running
        while (opModeIsActive())
        {
            //If its the firstTime block will equal the output of the queryTensorFlow method.
            if(firstTime)
            {
                telemetry.addData("firstTime = ", firstTime);
                telemetry.update();

                block = queryTensorFlow();
                //Queries the tensorFlow engine to find the block

                //they can all act as forward + something else ???
                switch (block) {
                    case 1:
                        //insert values for moving center
                        telemetry.addData("Block is on the ", " Center");
                        telemetry.update();

                        //movement code here

                        break;

                    case 2:
                        //insert values for moving right
                        telemetry.addData("Block is on the ", " Right");
                        telemetry.update();

                        //movement code here

                        break;

                    case 3:
                        //insert values for moving left
                        telemetry.addData("Block is on the ", " Left");
                        telemetry.update();

                        //movement code here


                        break;

                    default:
                        //insert code if something goes wrong
                        telemetry.addData("Block", "Not Found");
                        telemetry.update();
                        break;
                }
            }
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
    private void actuatorMove(double speed, double inches, boolean directionUp )
    {
        //Set up a new target variable
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            if(directionUp)
            {
                newTarget = robot.linearSlide.getCurrentPosition() + (int) (inches * robot.linearCPI);
            }
            else
            {
                newTarget = robot.linearSlide.getCurrentPosition() - (int) (inches * robot.linearCPI);
            }

            //Passes this target
            robot.linearSlide.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.linearSlide.setPower(Math.abs(speed));


            // keep looping while we are still active, and the motor is running.
            while (opModeIsActive() && robot.linearSlide.isBusy())
            {
                // Display it for the driver.
                telemetry.addData("Actuator moving to", newTarget);
                telemetry.addData("Actuator at", robot.linearSlide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.linearSlide.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    /**
     * Queries the tensorFlow engine to find the block
     * No parameter but returns a int to let program know where the block is
     * 1 = center
     * 2 = right
     * 3 = left
     *
     * Notes: want to integrate confidence reading
     *        max y values
     *        max timeout ( if timeout passes ends the code and defaults to center
     **/
    private int queryTensorFlow()
    {
        while (opModeIsActive() )
        {
            //Drive forward and turn around
            //encoderRobotDrive(0.5, 10, 10);
            //encoderRobotDrive(0.5, -34.558, 34.558);

            //int to determine gold block location: 1 = center, 2 = right, 3 = left
            int goldLocation = 0;

            if (opModeIsActive())
            {
                //Call the init TFod method to get block detection ready
                initTfod();
                //Activate Tensor Flow Object Detection.
                if (robot.tfod != null)
                {
                    robot.tfod.activate();
                    sleep(500);
                }

                while (opModeIsActive())
                {
                    if (robot.tfod != null)
                    {
                        List<Recognition> valuesForNerds = robot.tfod.getUpdatedRecognitions();
                        if (valuesForNerds != null)
                        {
                            telemetry.addData("value", valuesForNerds);
                            telemetry.update();
                        }
                        // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
                        List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            for (Recognition recognition : updatedRecognitions)
                            {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                                {
                                    telemetry.addData("value", "Center");
                                    telemetry.update();
                                    robot.tfod.shutdown();
                                    firstTime=false;

                                    //block is in the center

                                    return 1;
                                }
                                else
                                {
                                    //encoderRobotDrive(.5, 5,-5); //temp values
                                    telemetry.addData("Moving", "Right");
                                    telemetry.update();
                                    sleep(3000);
                                    List<Recognition> updatedRecognitions2 = robot.tfod.getUpdatedRecognitions();
                                    if (updatedRecognitions2 != null)
                                    {
                                        //noinspection LoopStatementThatDoesntLoop
                                        for (Recognition recognition2 : updatedRecognitions2)
                                        {
                                            if (recognition2.getLabel().equals(LABEL_GOLD_MINERAL))
                                            {
                                                telemetry.addData("value", "Right");
                                                telemetry.update();
                                                robot.tfod.shutdown();
                                                firstTime = false;

                                                //block on the right
                                                return 2;
                                            }
                                            else
                                            {
                                                telemetry.addData("value", "Left");
                                                telemetry.update();
                                                robot.tfod.shutdown();
                                                firstTime = false;

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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //Instantiate the Vuforia engine
        robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    //Method to init the tfod engine
    private void initTfod() {
        //
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        //
        robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, robot.vuforia);

        //Loads this years assets
        robot.tfod.loadModelFromAsset(robot.TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, robot.LABEL_SILVER_MINERAL);
    }

    //End of class
}