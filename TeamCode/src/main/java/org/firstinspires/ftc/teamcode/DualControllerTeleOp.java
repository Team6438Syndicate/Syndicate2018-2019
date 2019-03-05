package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Dual Controller TeleOp", group = "TeleOp 6438")
public class DualControllerTeleOp extends OpMode
{
   //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void init()
    {
        //init the hardware
        robot.init(hardwareMap);

        //Reset encoders
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Drive Motors should drive without encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Add stuff for the actuator and intake slide


        //telemetry
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        //Variables for power
        double leftPower, rightPower;
        double leftIntakePower, rightIntakePower;
        double linearActuatorPower = 0;


        //Logic for the intake
        if(gamepad2.right_trigger > .01)
        {
            leftIntakePower = gamepad2.right_trigger;
            rightIntakePower = gamepad2.right_trigger;
        }
        else if (gamepad2.left_trigger > .01)
        {
            leftIntakePower = gamepad2.left_trigger;
            rightIntakePower = -gamepad2.left_trigger;
        }
        else
        {
            leftIntakePower = 0;
            rightIntakePower = 0;
        }


        //Logic for the motors
        if( (gamepad1.left_stick_y > .01 || gamepad1.left_stick_y < .01) || (gamepad1.right_stick_y >.01 || gamepad1.right_stick_y < .01) )
        {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad2.right_stick_y;
        }
        else if (gamepad1.left_trigger > .01)
        {
            leftPower = gamepad1.left_trigger;
            rightPower = gamepad1.right_trigger;
        }
        else if (gamepad1.right_trigger > .01)
        {
            leftPower = gamepad1.right_trigger;
            rightPower = gamepad1.right_trigger;
        }
        else
        {
            leftPower = 0;
            rightPower = 0;
        }

        //Logic for moving the intake
        if ( (gamepad2.dpad_up && robot.intakeMover.getCurrentPosition() < robot.intakeMax ) )      //If the dpad is pressed and the intakeMover is at less than max move it up
        {
            intakeMove( 1,robot.intakeMover.getCurrentPosition() + 100);
        }
        else if ( (gamepad2.dpad_down && robot.intakeMover.getCurrentPosition() > robot.intakeMinimum) )
        {
            intakeMove(1,robot.intakeMover.getCurrentPosition() - 100);
        }

        //Logic for extending and retracting the slide
        if ( (gamepad2.dpad_left && robot.intakeSlide.getCurrentPosition() < robot.slideUnExtended ) )
        {
            intakeSlide(1,robot.slideExtended);
        }
        else if ( (gamepad2.dpad_right && robot.intakeSlide.getCurrentPosition() > robot.slideExtended))
        {
            intakeSlide(1,robot.slideUnExtended);
        }

        //Linear Actuator is controlled by the right stick up and down
        linearActuatorPower = -gamepad2.right_stick_y;

        //Sets the power to the motors
        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);
        robot.linearActuator.setPower(linearActuatorPower);
        robot.leftIntake.setPower(leftIntakePower);
        robot.rightIntake.setPower(rightIntakePower);

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

        while (robot.intakeMover.isBusy())
        {
            telemetry.addData("Currently at ", robot.intakeMover.getTargetPosition());
            telemetry.addData("Going to ", robot.intakeMover.getCurrentPosition());
            telemetry.update();
        }
    }

    //Method to move the intake slide
    private void intakeSlide(double speed, int position)
    {
        //Set the target
        robot.intakeSlide.setTargetPosition(position);

        //Turn On RUN_TO_POSITION
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start motion
        robot.intakeSlide.setPower(Math.abs(speed));

        while (robot.intakeSlide.isBusy())
        {
            telemetry.addData("Moving to", robot.intakeSlide.getTargetPosition());
            telemetry.addData("Currently At", robot.intakeSlide.getCurrentPosition());
            telemetry.update();
        }
    }

    //Method to hang the robot no params and returns
    private void robotHang()
    {
        //Sets the target position to 0 (retracted)
        robot.linearActuator.setTargetPosition(0);

        //Sets the motor to run to position
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motor to full power
        robot.intakeMover.setPower(1);

        //While the motor is performing actuation add telemetry to show the user
        while(robot.linearActuator.isBusy())
        {
            telemetry.addData("Robot Status: ", "Hanging");
            telemetry.addData("Going to ", robot.intakeMover.getTargetPosition());
            telemetry.addData("Currently at ", robot.intakeMover.getCurrentPosition());
            telemetry.update();
        }
    }
}
