package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(name = "Dual Controller Test", group = "TeleOp 6438")
public class DualControllerTest extends OpMode
{
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    //Creating the intake power variable
    private double intakePower = 0;

    @Override
    public void init()
    {
        //init the hardware
        robot.init(hardwareMap);

        //Reset encoders
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Drive Motors should drive without encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //telemetry
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        //Variables for power
        double leftPower, rightPower;

        //Logic for the intake
        if( gamepad2.a && intakePower == 0 )                                                        //Tested
        {
            intakePower = 1;
        }
        else if ( gamepad2.a && intakePower != 0)                                                   //Tested
        {
            intakePower = 0;
        }
        else if (gamepad2.b)                                                                        //Tested
        {
            intakePower = -1;
        }

        //Logic for the intake slide
        if (gamepad2.dpad_right && robot.intakeSlide.getCurrentPosition() < robot.slideExtended && (robot.linearActuator.getCurrentPosition() < robot.laInterference) )         //Untested
        {
            intakeSlide(1,robot.slideExtended);
        }
        else if (gamepad2.dpad_left && (robot.intakeSlide.getCurrentPosition() > robot.slideUnExtended) ) //Untested
        {
            intakeSlide(.3,robot.slideUnExtended);
        }
        else if (gamepad2.dpad_up && robot.intakeSlide.getCurrentPosition() != robot.slideExtended  && (robot.linearActuator.getCurrentPosition() < robot.laInterference) && robot.intakeMover.getCurrentPosition() > 300 )        //Untested
        {
            intakeSlide(1,robot.intakeSlide.getCurrentPosition() + 100);
        }
        else if (gamepad2.dpad_down && robot.intakeSlide.getCurrentPosition() == robot.slideUnExtended && (robot.intakeMover.getCurrentPosition() > 300) )     //Untested
        {
            intakeSlide(1,robot.intakeSlide.getCurrentPosition() - 100);
        }

        //Logic for intake itself
        if( (gamepad2.x && !gamepad1.x) ||  (gamepad1.x && !gamepad2.x) )                           //Tested
        {
            telemetry.addData("TO COMPLETE PROCESS: ", "PRESS DOWN BOTH X BUTTONS");
            telemetry.update();
        }
        if (gamepad2.x && gamepad1.x && robot.linearActuator.getCurrentPosition() > robot.laInterference)           //Untested
        {
            telemetry.addData("LOWER ACTUATOR BEFORE BEGINNING ","ASAP");             //Untested
        }
        else if (gamepad2.x && gamepad1.x && robot .linearActuator.getCurrentPosition() < robot.laInterference )   //Untested
        {
            intakeMove(1,robot.intakeOutPosition);
        }

        if(gamepad2.right_trigger > .05 && robot.intakeMover.getCurrentPosition() != robot.intakeMinimum && robot.linearActuator.getCurrentPosition() > robot.laInterference)     //untested
        {
            intakeMove(1,robot.intakeMover.getCurrentPosition() + 100);
        }
        else if (gamepad2.left_trigger > .05 && robot.intakeMover.getCurrentPosition() < robot.intakeMax && robot.linearActuator.getCurrentPosition() > robot.laInterference )  //untested
        {
            intakeMove ( 1, robot.intakeMover.getCurrentPosition() - 100);
        }
        else if (gamepad2.right_bumper)
        {
            intakeMove(.5,robot.intakeMover.getCurrentPosition());
        }

        //Control for the linear actuator
        double linearActuatorPower = gamepad2.left_stick_y;                                         //Could need to be negated

        //Controls for tank treads
        if( gamepad1.right_trigger > .01)                                                           //Tested
        {
            leftPower = gamepad1.right_trigger;
            rightPower = gamepad1.right_trigger;
        }
        else if ( gamepad1.left_trigger > .01)                                                      //Tested
        {
            leftPower = -gamepad1.left_trigger;
            rightPower = -gamepad1.left_trigger;
        }
        else                                                                                        //Tested
        {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        }

        robot.linearActuator.setPower(linearActuatorPower);

        robot.leftIntake.setPower(intakePower);
        robot.rightIntake.setPower(intakePower);

        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);

        //Telemetry to constantly refresh data to update user
        telemetry.addData("Left Power: ", leftPower);
        telemetry.addData("Right Power: ", rightPower);
        telemetry.addData("Intake Servos Power:", intakePower );
        telemetry.addData("Linear Actuator Power ", linearActuatorPower);
        telemetry.addData("Linear Actuator Position ", robot.linearActuator.getCurrentPosition());
        telemetry.addData("Intake Mover Power", robot.intakeMover.getCurrentPosition());
        telemetry.addData("Intake Slide Power ", robot.intakeSlide.getCurrentPosition());
        telemetry.update();

    }

    //Method to move the intake
    private void intakeMove(double speed, int position)
    {
        //Set the target
        robot.intakeMover.setTargetPosition(position);

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


}
