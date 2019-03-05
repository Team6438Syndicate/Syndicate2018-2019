/**
 * Name: Team6438TeleOpTest
 * Purpose: This class allows the robot to run during the Driver Controlled period
 *          This class can: move the driver motors, move the intake spinner,
 *          move the Linear Actuator, and spin the intake.
 * Author: Matthew Batkiewicz
 * Contributors: Bradley Abelman, Matthew Kaboolian
 * Creation: 11/8/18
 * Last Edit: 1/20/19
 * Additional Notes: Needs to be cleaned up
 **/
package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Deprecated
//@Disabled    //Uncomment this if the op mode needs to not show up on the DS
@TeleOp(name = "Team 6438 Driver Controlled (Test) ", group = "Team 6438 TeleOp (Test)")
public class TeleOp6438Test extends OpMode
{

    //Create reference to the Team6438HardwareMap Class
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    //Variables for intake location
    int tuckedPosition = 1;
    int verticalPosition = 1;
    int downPosition = 1;

    //Positions for the slide
    int extendedSlide = 1;
    int retractedSlide = 1;

    //Boolean to determine if were in linear Mode
    public boolean linearMode = false;

    //Length that the intake slide wire needs to be pulled
    final double slideWire = 8.0;

    //Boolean to assess if the arm should be allowed to move if true the arm can move if not it can't
    private boolean run = true;

    @Override
    public void init()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);

        //Init and map the hardware
        robot.init(hardwareMap);

        //Resets the encoders on the intake to allow for movement
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the drive motors to run without encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Telemetry to show user robot has been init
        telemetry.addData("Hardware Status ", "Mapped");
        telemetry.update();
    }

    @Override
    public void start()
    {
        //When the opmode starts tuck the servos in to prevent damage during gameplay
        robot.teamMarkerServo.setPosition(robot.tucked);
        robot.cameraMount.setPosition(robot.cameraMountTucked);
    }
    @Override
    public void loop()
    {
        //Declaring power variables
        double leftPower = 0;
        double rightPower = 0;
        double linearActuatorPower;
        double intakeSpinnerPower;
        double intakeSlidePower;

        //Linear Actuator power is set to the left y stick
        linearActuatorPower = gamepad2.left_stick_y;

        //Intake spinner is the right gamepad 2  stick up and down
        intakeSpinnerPower = gamepad2.right_stick_y;

        //Gamepad logic to move the intake
        ///When A is pressed go to the down position
        if (gamepad2.a)
        {
            //robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Intake move method invocation
            intakeMove(.5,  1300);

            //Should be able to be deleted
            robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        /*
         * Gamepad logic to extend/retract the intake
         */
        ///When Y is pressed extend to max position
        /*if (gamepad2.y)
         *{
         *    intakeSlide(.75, -1);
         *}*/

        //robot.intakeMover.setPower(-gamepad2.left_trigger);

        /*if (gamepad2.x)
        {
            intakeMove(1, -1300);
        }*/

        if(gamepad2.dpad_left)
        {
            int target = robot.intakeMover.getCurrentPosition()+50;
            intakeMove(.75,target);
        }
        if(gamepad2.dpad_right)
        {
            int target = robot.intakeMover.getCurrentPosition()-50;
            intakeMove(.75,target);
        }


        if (gamepad2.left_bumper)
        {
            //int increment = (int) (10 * gamepad2.left_trigger);
            //moveIntake(.5, -1);
            intakeMove(0.75, -350);
            //robot.intakeMover.setPower(0);
        }
        if (gamepad2.right_bumper)
        {
            //int increment = (int) (10 * gamepad2.left_trigger);
            //moveIntake(.5, -1);
            intakeMove(0.75, 0);
            //robot.intakeMover.setPower(0);
        }

        if (gamepad1.left_bumper)
        {
            //
            leftPower =  -gamepad1.left_stick_y;
            rightPower = -gamepad1.left_stick_y;
            telemetry.addData("Linear Mode", "Engaged");
        }
        else
        {
            leftPower=   -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        }

        //Sending the power info to the motors
        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);
        robot.linearActuator.setPower(linearActuatorPower);
        //robot.intakeSpinner.setPower(intakeSpinnerPower);
        //robot.intakeSlide.setPower(intakeSlidePower);

        //Telemetry
        //telemetry.addData("Trigger: ", gamepad2.right_trigger);
        telemetry.addData("Left Power: ", leftPower);
        telemetry.addData("Right Power: ", rightPower);
        telemetry.addData("Linear Actuator Power: ", linearActuatorPower);
        telemetry.addData("Intake Power: ", intakeSpinnerPower);
        //telemetry.addData("Intake Slide Power", intakeSlidePower);
        //telemetry.addData("Intake Slide Position", robot.intakeSlide.getCurrentPosition());
        //telemetry.addData("Linear Mode Enabled: " , " " + linearMode);
        telemetry.addData("Linear Actuator Position: ", robot.linearActuator.getCurrentPosition());
        telemetry.addData("Intake Mover Currently At: ", robot.intakeMover.getCurrentPosition());
        telemetry.addData("Speed", robot.imu.getVelocity());
        //telemetry.addData("Arm Linear Slide Currently At: ", robot.intakeSlide.getCurrentPosition());
        telemetry.update();
    }

    //Method to move the intake
    private void intakeMove(double speed, double position)
    {
        int newTarget = (int) (position);

        // Determine new target position, and pass to motor controller
        //newTarget = robot.intakeMover.getCurrentPosition() + (int)(position);

        //Set the target
        robot.intakeMover.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        robot.intakeMover.setPower(Math.abs(speed));

        while (robot.intakeMover.isBusy())
        {
            telemetry.addData("Currently at ", robot.intakeMover.getCurrentPosition());
            telemetry.addData("Going to ", robot.intakeMover.getTargetPosition());
            telemetry.update();
        }
    }

    private void intakeSlide(double speed, int position)
    {
        //Temp var
        int slideTarget;

        //Sets the slide position to max length
        slideTarget = (int)((Math.round(slideWire))*(position));

        //Set the target
        robot.intakeSlide.setTargetPosition(slideTarget);

        //Turn On RUN_TO_POSITION
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start motion
        robot.intakeSlide.setPower(Math.abs(speed));

        while (robot.intakeSlide.isBusy())
        {
            telemetry.addData("Moving to", robot.intakeSlide.getTargetPosition());
            telemetry.addData("Currently at", robot.intakeSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}
