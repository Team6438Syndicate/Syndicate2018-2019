package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Encoder Reading",group = "Test TeleOp")
public class TelemetryOpMode extends OpMode
{
    //Reference to our robot
    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void init()
    {
        //Init hardware
        robot.init(hardwareMap);

        //Motor Logic
        robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Intake Mover Zero Power Behavoir", robot.intakeMover.getZeroPowerBehavior());
        telemetry.addData("Intake Slide Zero Power Behavoir", robot.intakeSlide.getZeroPowerBehavior());

        //Telemetry to let user know the hardware map is done
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        telemetry.addData("Linear Actuator Current Position: ", robot.linearActuator.getCurrentPosition());
        telemetry.addData("Linear Slide Current Position: ", robot.intakeSlide.getCurrentPosition());
        telemetry.addData("Intake Mover Current Position: ", robot.intakeMover.getCurrentPosition());
        telemetry.update();

        if(gamepad1.x)
        {
            //Floats the motors
            robot.intakeMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(gamepad1.y)
        {
            //Brakes the motors
            robot.intakeMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gamepad1.a)
        {
            robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Reset","Reset");
            telemetry.update();
        }

    }
}
