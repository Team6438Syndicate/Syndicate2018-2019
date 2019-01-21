/**
 * Name: Team6438TeleOp
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Team 6438 Driver Controlled", group = "Team 6438 TeleOp")
public class TeleOp6438 extends OpMode
{
    //Create reference to the Team6438HardwareMap Class
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    //Variables for intake location
    int tuckedPosition = 1;
    int verticalPosition = 1;
    int downPosition = 1;

    //Boolean to asses if the arm should be allowed to move if true the arm can move if not it can't
    private boolean run = true;

    @Override
    public void init()
    {
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
    public void loop()
    {
        //Declaring power variables
        double leftPower, rightPower;
        double linearSlidePower;
        double intakeSpinnerPower;

        //Left power is the left stick up and down
        leftPower = -gamepad1.left_stick_y;

        //Right power is the right stick up and down
        rightPower = -gamepad1.right_stick_y;

        //Linear slide is the second gamepad left stick up and down
        linearSlidePower = gamepad2.left_stick_y;

        //Intake spinner is the right gamepad 2  stick up and down
        intakeSpinnerPower = gamepad2.right_stick_y;

        //Sending the power info to the motors
        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);
        robot.linearSlide.setPower(linearSlidePower);
        robot.intakeSpinner.setPower(intakeSpinnerPower);

        //Telemetry
        telemetry.addData("Left Power:", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Linear Slide Power", linearSlidePower);
        telemetry.addData("Intake Power", intakeSpinnerPower);
        telemetry.addData("Going to ", robot.intakeMover.getTargetPosition());
        telemetry.addData("Currently At", robot.intakeMover.getCurrentPosition());
        telemetry.update();

        /*
         * Gamepad logic to move the intake
         */
        ///When A is pressed go to the down position
        if (gamepad2.a)
        {
            robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intakeMove(.5,  -1000);


            while ( robot.intakeMover.isBusy() )
            {
                telemetry.addData("Going to ", robot.intakeMover.getTargetPosition());
                telemetry.addData("Currently At", robot.intakeMover.getCurrentPosition());
                telemetry.update();
            }
            robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }

        if(run)
        {
            if (gamepad2.right_trigger>.01)
            {
                intakeMove(.5,400);
                run = false;
            }
        }
        else if (gamepad2.left_trigger>.01)
        {
            //int increment = (int) (10 * gamepad2.left_trigger);
            //moveIntake(.5, -1);
            intakeMove(.5,50);
            robot.intakeMover.setPower(0);
            run = true;
        }
    }

    //Method to move the intake
    private void intakeMove(double speed, double position)
    {
        int newTarget;

        // Determine new target position, and pass to motor controller
        newTarget = robot.intakeMover.getCurrentPosition() + (int)(position);

        //Set the target
        robot.intakeMover.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        robot.intakeMover.setPower(Math.abs(speed));
    }
}
