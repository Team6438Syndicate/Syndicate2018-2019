package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(name = "Mecanum Full", group = "TeleOp 6438")
public class MecanumFullMode extends OpMode {
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void init() {
        //init the hardware
        robot.leftFrontMotor        = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        robot.rightFrontMotor       = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        robot.leftRearMotor         = hardwareMap.get(DcMotor.class, "leftRearDrive");
        robot.rightRearMotor        = hardwareMap.get(DcMotor.class, "rightRearDrive");

        //Reset encoders

        //Drive Motors should drive without encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telemetry
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Variables for power
        double fLPower, fRPower, rLPower, rRPower;

        //Controls for tank treads
        if (gamepad1.left_bumper) {
            fLPower = -1;
            fRPower = -0.2;
            rLPower = -1;
            rRPower = -0.2;
        }
        else if (gamepad1.right_bumper) {
            fLPower = -0.2;
            fRPower = 1;
            rLPower = -0.2;
            rRPower = 1;
        }
        else {
            fLPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
            fRPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            rLPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            rRPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        }

        robot.leftFrontMotor.setPower(fLPower);
        robot.rightFrontMotor.setPower(fRPower);
        robot.leftRearMotor.setPower(rLPower);
        robot.rightRearMotor.setPower(rRPower);

        //Telemetry to constantly refresh data to update user
        telemetry.addData("Front Left Power: ", fLPower);
        telemetry.addData("Front Right Power: ", fRPower);
        telemetry.addData("Rear Left Power: ", rLPower);
        telemetry.addData("Rear Right Power: ", rRPower);
        telemetry.update();
    }

}