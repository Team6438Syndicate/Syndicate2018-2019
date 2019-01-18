package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

//import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

//TO DO
// Find linearCPI
// Test encoder based driving
// Integrate way to control linearSlide in OpMode
//Distance from landing to gems: Approximately 34 inches
//Height of bracket off the ground: 19 inches
//test

@Autonomous(name = "Run It Now Depot", group = "Blue Autonomous 6438")
public class RunItNowDepot extends LinearOpMode {
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    //Variables for TensorFlow
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    //Variable to store where the block is
    private static String blockLocation=null;

    //Constructor method to receive the parameters

    @Override
    public void runOpMode() throws InterruptedException {
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

        String blockLocation = null;
        boolean checkObjects = true;
        //Wait for the start button to be pressed by the driver

        initVuforia();

        waitForStart();

        //While the program is running
        while (opModeIsActive())
        {
        /*
            actuatorMove(1, 28);

            //Build Everything

            //Drop the linear actuator
            actuatorMove(0.5, -8);

            //Twist the servo
            robot.hookServo.setPower (-1);
            sleep(500);
            robot.hookServo.setPower(0);
            */

        //Drive forward and turn around
            //encoderRobotDrive(0.5, 10, 10);
            //encoderRobotDrive(0.5, -34.558, 34.558);
            
            
/*
            if (checkObjects)
            {
               blockLocation();
                sleep(1000);
                checkObjects = false;
            }
*/

            //int to determine gold block location: 1 = center, 2 = right, 3 = left
            int goldLocation = 0;

           if (opModeIsActive())
        {
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
                        telemetry.addData(String.valueOf(updatedRecognitions.size()), "");
                        telemetry.addData("robot", robot.tfod.getUpdatedRecognitions());
                        //telemetry.addData("list size", robot.tfod.getUpdatedRecognitions().size());
                        telemetry.update();

                        sleep(50000);

                        for (Recognition recognition : updatedRecognitions)
                        {
                            if(recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                            {
                                //center
                                goldLocation = 1;
                                telemetry.addData("value","Center");
                                telemetry.update();
                                robot.tfod.shutdown();
                            }
                            else
                            {
                                //encoderRobotDrive(.5,100,-100); //temp values
                                sleep(3000);
                                List<Recognition> updatedRecognitions2 = robot.tfod.getUpdatedRecognitions();
                                if (updatedRecognitions2 != null)
                                {
                                    for (Recognition recognition2 : updatedRecognitions2)
                                    {
                                        if(recognition2.getLabel().equals(LABEL_GOLD_MINERAL))
                                        {
                                            //right
                                            goldLocation = 2;
                                            telemetry.addData("value","Right");
                                            telemetry.update();
                                            robot.tfod.shutdown();
                                        }
                                        else
                                        {
                                            //left
                                            goldLocation = 3;
                                            telemetry.addData("value","Left");
                                            telemetry.update();
                                            robot.tfod.shutdown();
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
        
        //Tells the robot what path to take
        if (goldLocation == 1) //Center
        {
            
        }
        
        if (goldLocation ==2) //Right
        {
            
        }
        
        if (goldLocation == 3) //Left
        {
            
        }
            
            //Team marker throw
            robot.teamMarkerServo.setPower (1);
            sleep (1000);
            robot. teamMarkerServo.setPower (0);

            //Drive to depot to deposit marker
            //insert code to do this

            //Drive back towards the crater for crater park
            //insert code to do this
            
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
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newTarget = robot.linearSlide.getCurrentPosition() + (int) (inches * robot.linearCPI);

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
    private void intakeMove(double speed, String position)
    {

    }


    //Method to query the tensorFlow Engine
   /* public String blockLocation()
    {
        //initVuforia();

        if (opModeIsActive())
        {
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
                    // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    for (Recognition recognition : updatedRecognitions)
                    {
                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                        {
                            //center
                            telemetry.addData("value","a");
                            telemetry.update();
                            robot.tfod.shutdown();
                        }
                        else
                        {
                            //encoderRobotDrive(.5,100,-100); //temp values
                            sleep(3000);
                            updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                            for (Recognition recognition2 : updatedRecognitions)
                            {
                                if(recognition2.getLabel().equals(LABEL_GOLD_MINERAL))
                                {
                                    //right
                                    telemetry.addData("value","b");
                                    telemetry.update();
                                    robot.tfod.shutdown();

                                }
                                else
                                {
                                    //left
                                    telemetry.addData("value","c");
                                    telemetry.update();
                                    robot.tfod.shutdown();

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
            return "none";
        }
        return null;
    }
*/
    //Method to init the vuforia engine
    private void initVuforia() {

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //Sets the vuforia key
        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;

        //Sets camera direction
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Uncomment this in order to use an external webcam.
         * parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
         **/

        //Instantiate the Vuforia engine
        robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Loading trackables is not necessary for the Tensor Flow Object Detection engine.
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
}
