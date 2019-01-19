package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This is the TeleOp Class for Team 6438.
 * Created by Team 6438 Programmers on 11/15/18
 * Last edit: 1/3/19
 */
@TeleOp(name = "Team 6438 Driver Controlled", group = "Team 6438 TeleOp")
public class TeleOp6438 extends OpMode
{
    //Init the hardwareMap
    Team6438HardwareMap robot = new Team6438HardwareMap();

    //Variables for intake location

    boolean run = true;

    /*
     *   Use hardware the normal way just add robot before it
     *    i.e. robot.leftDrive.setPower() as opposed to leftDrive.setPower
     *   Everything is mapped to a central class so there are no name discrepancies
     *
     */

    @Override
    public void init()
    {
        //Init and map the hardware
        robot.init(hardwareMap);

        //Turns off color sensor Led (DEPRECIATED)
        //robot.colorSensor.enableLed(false);


        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        double right;

        /*
         * Declaring values for said power variables
         */

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
        telemetry.addData("trigger", gamepad2.right_trigger);
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

            encoderDrive(.5,  -1000);


            while ( robot.intakeMover.isBusy() )
            {
                telemetry.addData("Going to ", robot.intakeMover.getTargetPosition());
                telemetry.addData("Currently At", robot.intakeMover.getCurrentPosition());
                telemetry.update();
            }
            robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }

        if(run){
            if (gamepad2.right_trigger>.01)
            {
                //int increment = (int) (10 * gamepad2.right_trigger);

                //moveIntake(.5,1);

                encoderDrive(.5,  400);
                run = false;
            }
        }


        else if (gamepad2.left_trigger>.01)
        {
            //int increment = (int) (10 * gamepad2.left_trigger);
            //moveIntake(.5, -1);
            encoderDrive(.5,  50);
            robot.intakeMover.setPower(0);
            run = true;

        }

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double position)
    {
        int newTarget;

        // Ensure that the opmode is still active


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
