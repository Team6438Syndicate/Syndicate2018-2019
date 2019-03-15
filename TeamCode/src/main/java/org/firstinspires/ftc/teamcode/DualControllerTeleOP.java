package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(name = "Dual Controller TeleOp", group = "TeleOp 6438")
public class DualControllerTeleOP extends OpMode
{
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    //Creating the intake power variable
    private double intakePowerL = 0;
    private double intakePowerR = 0;

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
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //telemetry
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        //Variables for power
        double fLPower, fRPower, rLPower, rRPower;

        //Logic for the intake
        if( gamepad2.a && intakePowerL == 0 && intakePowerR == 0 )                                                        //Tested
        {
            intakePowerL = 1;
            intakePowerR = 1;
        }
        else if ( gamepad2.y)                                                                       //Tested
        {
            intakePowerL = 0;
            intakePowerR = 0;
        }
        else if (gamepad2.b)                                                                        //Tested
        {
            intakePowerL = -1;
            intakePowerR = -1;
        }
        else if( gamepad1.a)
        {
            intakePowerL = 0.5;
            intakePowerR = -0.5;
        }
        else if ( gamepad1.b)
        {
            intakePowerL = -0.5;
            intakePowerR = 0.5;
        }

        //Logic for the intake slide
        if (gamepad2.dpad_right && robot.intakeSlide.getCurrentPosition() < robot.slideExtended)         //Untested
        {
            intakeSlide(1,robot.slideExtended);
        }
        else if (gamepad2.dpad_left && (robot.intakeSlide.getCurrentPosition() > robot.slideUnExtended) ) //Untested
        {
            intakeSlide(.3,robot.slideUnExtended);
        }
        else if (gamepad2.dpad_up && robot.intakeSlide.getCurrentPosition() + 100 <= robot.slideExtended && robot.intakeMover.getCurrentPosition() > 300 )        //Untested
        {
            intakeSlide(1,robot.intakeSlide.getCurrentPosition() + 100);
        }
        else if (gamepad2.dpad_down && robot.intakeSlide.getCurrentPosition() - 100 >= robot.slideUnExtended && (robot.intakeMover.getCurrentPosition() > 300) )     //Untested
        {
            intakeSlide(1,robot.intakeSlide.getCurrentPosition() - 100);
        }
        else
        {
            robot.intakeSlide.setPower(0);
        }

        //Logic for intake itself
        if( (gamepad2.x && !gamepad1.x) ||  (gamepad1.x && !gamepad2.x) )                           //Tested
        {
            telemetry.addData("TO COMPLETE PROCESS: ", "PRESS DOWN BOTH X BUTTONS");
            telemetry.update();
        }
        if (gamepad2.x && gamepad1.x && robot.linearActuator.getCurrentPosition() == robot.laInterference)           //Untested
        {
            telemetry.addData("LOWER ACTUATOR BEFORE BEGINNING ","ASAP");             //Untested
        }
        else if (gamepad2.x && gamepad1.x && robot .linearActuator.getCurrentPosition() != robot.laInterference )   //Untested
        {
            intakeMove(1,robot.intakeOutPosition);
        }

        if(gamepad2.right_trigger > .05 && robot.intakeMover.getCurrentPosition() != robot.intakeMinimum && robot.linearActuator.getCurrentPosition() != robot.laInterference)     //untested
        {
            intakeMove(1,robot.intakeMover.getCurrentPosition() + 100);
        }
        else if (gamepad2.left_trigger > .05 && robot.intakeMover.getCurrentPosition() != robot.intakeMax && robot.linearActuator.getCurrentPosition() != robot.laInterference )  //untested
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
        if ( Math.abs(gamepad1.left_stick_y) > .075 && Math.abs(gamepad1.left_stick_x) > .075) {                                                                                       //Tested
            fLPower = -( (gamepad1.left_stick_y + gamepad1.left_stick_x)/Math.sqrt(2) );
            fRPower = -( (gamepad1.left_stick_y - gamepad1.left_stick_x)/Math.sqrt(2) );
            rLPower = -( (gamepad1.left_stick_y - gamepad1.left_stick_x)/Math.sqrt(2) );
            rRPower = -( (gamepad1.left_stick_y + gamepad1.left_stick_x)/Math.sqrt(2) );
        }
        else {
            fLPower = 0;
            fRPower = 0;
            rLPower = 0;
            rRPower = 0;
        }

        robot.linearActuator.setPower(linearActuatorPower);

        robot.leftIntake.setPower(-intakePowerL);
        robot.rightIntake.setPower(-intakePowerR);

        robot.leftFrontMotor.setPower(fLPower);
        robot.rightFrontMotor.setPower(fRPower);
        robot.leftRearMotor.setPower(rLPower);
        robot.rightRearMotor.setPower(rRPower);

        //Telemetry to constantly refresh data to update user
        telemetry.addData("Front Left Power: ", fLPower);
        telemetry.addData("Front Right Power: ", fRPower);
        telemetry.addData("Rear Left Power: ", rLPower);
        telemetry.addData("Rear Right Power: ", rRPower);
        telemetry.addData("Intake Servo Left Power:", intakePowerL );
        telemetry.addData("Intake Servo Right Power:", intakePowerR );
        telemetry.addData("Linear Actuator Power ", linearActuatorPower);
        telemetry.addData("Linear Actuator Position ", robot.linearActuator.getCurrentPosition());
        telemetry.addData("Intake Mover Position", robot.intakeMover.getCurrentPosition());
        telemetry.addData("Intake Slide Position ", robot.intakeSlide.getCurrentPosition());
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
