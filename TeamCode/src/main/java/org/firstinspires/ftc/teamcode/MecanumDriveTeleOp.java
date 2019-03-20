package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "Basic Mecanum Drive", group = "TeleOp 6438")
public class MecanumDriveTeleOp extends OpMode {
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void init() {
        //init the hardware
        robot.leftFrontMotor        = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        robot.rightFrontMotor       = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        robot.leftRearMotor         = hardwareMap.get(DcMotor.class, "leftRearDrive");
        robot.rightRearMotor        = hardwareMap.get(DcMotor.class, "rightRearDrive");
        robot.leftIntake            = hardwareMap.get(CRServo.class, "leftIntake");
        robot.rightIntake           = hardwareMap.get(CRServo.class, "rightIntake");

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
        double fLPower, fRPower, rLPower, rRPower,
                intakeLPower, intakeRPower;

        //Controls for tank treads
        if (gamepad1.right_trigger >= 0.1 && gamepad1.left_trigger == 0) {
            intakeLPower = 1;
            intakeRPower = 1;
        }
        else if (gamepad1.left_trigger >= 0.1 && gamepad1.right_trigger == 0) {
            intakeLPower = -1;
            intakeRPower = -1;
        }
        else {
            intakeLPower = 0;
            intakeRPower = 0;
        }


        fLPower = -(gamepad1.left_stick_y + gamepad1.left_stick_x);
        fRPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
        rLPower = -(gamepad1.left_stick_y - gamepad1.left_stick_x);
        rRPower = gamepad1.left_stick_y + gamepad1.left_stick_x;

        fLPower += -gamepad1.right_stick_x;
        fRPower += -gamepad1.right_stick_x;
        rLPower += -gamepad1.right_stick_x;
        rRPower += -gamepad1.right_stick_x;


        robot.leftFrontMotor.setPower(fLPower);
        robot.rightFrontMotor.setPower(fRPower);
        robot.leftRearMotor.setPower(rLPower);
        robot.rightRearMotor.setPower(rRPower);

        robot.leftIntake.setPower(intakeLPower);
        robot.rightIntake.setPower(intakeRPower);

        //Telemetry to constantly refresh data to update user
        telemetry.addData("Front Left Power: ", fLPower);
        telemetry.addData("Front Right Power: ", fRPower);
        telemetry.addData("Rear Left Power: ", rLPower);
        telemetry.addData("Rear Right Power: ", rRPower);
        telemetry.update();
    }

}