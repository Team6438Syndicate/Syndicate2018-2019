package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test", group = "TeleOp 6438")

public class servoTest extends OpMode {
    CRServo one, two;


    public void init() {
        one = hardwareMap.crservo.get("one");
        two = hardwareMap.crservo.get("two");
    }

    public void loop() {
       double intakeLPower, intakeRPower;

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

        one.setPower(intakeLPower);
        two.setPower(-intakeRPower);

    }
}