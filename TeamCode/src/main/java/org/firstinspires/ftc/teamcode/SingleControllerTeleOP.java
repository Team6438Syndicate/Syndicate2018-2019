package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Controller TeleOp", group = "TeleOp 6438")
public class SingleControllerTeleOP extends OpMode
{
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void init()
    {
        //Init the hardware
        robot.init(hardwareMap);

        //Reset encoders
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Drive Motors should drive without encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Add stuff for the actuator and intake slide


        //Telemetry to let use know the hardware has been successfully mapped
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();

    }

    @Override
    public void start()
    {
        //Ensures all servos are put away to ensure prevention from damage
        robot.cameraMount.setPosition(robot.cameraMountTucked);
        robot.teamMarkerServo.setPosition(robot.tucked);

        //in case tfod is on shut it down to prevent CPU overloading
        if(robot.tfod != null)
        {
            robot.tfod.shutdown();
        }
    }

    @Override
    public void loop()
    {
        /*
         * Logic for drive motors
         */
        //Variables for power
        double leftPower, rightPower;

        //Assigning said powers
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);

        /*
          Control for actuator
         */
        if(gamepad1.left_stick_x > .25 || gamepad1.left_stick_x < -.25)
        {
            robot.linearActuator.setPower(-gamepad1.left_stick_x);
            telemetry.addData("Linear Actuator Power: ", robot.linearActuator.getPower());
            telemetry.update();
        }
        else
        {
            robot.linearActuator.setPower(0);
        }
        if(gamepad2.y)
        {
            robotHang();
        }


        /*
          Intake Spinner Logic
         */
        //Logic to control the spinner at the end of the intake
        if ( gamepad1.a )
        {
            intakeSpin(1);
        }
        else if ( gamepad1.b )
        {
            intakeSpin(-1);
        }
        else
        {
            intakeSpin(0);
        }

        /*
         * Intake Slide logic
         */
        if(gamepad1.dpad_left && (robot.intakeSlide.getCurrentPosition() < robot.slideExtended) && (robot.linearActuator.getCurrentPosition() < 16000) )
        {
            intakeSlide(.5, robot.slideExtended);
        }
        else if (gamepad1.dpad_right && (robot.intakeSlide.getCurrentPosition() > robot.slideUnExtended) && (robot.linearActuator.getCurrentPosition() < 16000) )
        {
            intakeSlide(.5,robot.slideUnExtended);
        }
        else if ( gamepad1.right_stick_x > .25 || gamepad1.right_stick_x > -.25)
        {
            robot.intakeSlide.setPower(gamepad1.right_stick_x);
        }

        /*
          Intake Move logic
         */
        //Logic for moving the intake based on triggers
        if(gamepad1.left_trigger > 0.05)
        {
            intakeMove(1,gamepad1.left_trigger * 65);
        }
        else if (gamepad1.right_trigger > 0.05)
        {
            intakeMove(1, -gamepad1.right_trigger * 65);
        }
        else if (gamepad1.dpad_up && (robot.intakeMover.getCurrentPosition() != robot.intakeDunk) )
        {
            intakeMove(1,robot.intakeDunk);
        }
        else if (gamepad1.dpad_down && (robot.intakeMover.getCurrentPosition() != robot.intakeFloor))
        {
            intakeMove(1,robot.intakeFloor);
        }
        //If X is pressed pop the intake out
        else if(gamepad1.x && robot.intakeMover.getCurrentPosition() == 0)
        {
            intakeMove(1,robot.intakeOutPosition);
        }


    }

    //Method to spin the intake
    private void intakeSpin (double power)
    {
        robot.leftIntake.setPower(power);

        //one of these might have to be negative power
        robot.rightIntake.setPower(-power);
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
