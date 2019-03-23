package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "Mecanum Full", group = "TeleOp 6438")
public class MecanumFullMode extends OpMode {
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();
    private boolean fullSpeed = false;
    private DistanceSensor sensorRange;
    private DcMotor intakeSlide;
    private double powerFactor = 1;

    @Override
    public void init() {
        //init the hardware
        robot.leftFrontMotor        = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        robot.rightFrontMotor       = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        robot.leftRearMotor         = hardwareMap.get(DcMotor.class, "leftRearDrive");
        robot.rightRearMotor        = hardwareMap.get(DcMotor.class, "rightRearDrive");


        //this is test for now
        intakeSlide = hardwareMap.dcMotor.get("linearSlide");

        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

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


    //read this https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html

    @Override
    public void loop() {
        //Variables for power
        double fLPower, fRPower, rLPower, rRPower;

        //more test
        double linearSlidePower = gamepad2.right_stick_y;
        intakeSlide.setPower(linearSlidePower);

        if (gamepad2.x)
        {
            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeSlide.setTargetPosition(intakeSlide.getCurrentPosition());

            intakeSlide.setPower(.5);

            while(intakeSlide.isBusy())
            {
                telemetry.addData("Moving to ", intakeSlide.getTargetPosition());
                telemetry.addData("Currently At", intakeSlide.getCurrentPosition());
                telemetry.update();
            }
        }

        //Controls for tank treads
        if (gamepad1.left_bumper) {
            fLPower = 0.2;
            fRPower = 1;
            rLPower = 0.2;
            rRPower = 1;
        }
        else if (gamepad1.right_bumper) {
            fLPower = -1;
            fRPower = -0.2;
            rLPower = -1;
            rRPower = -0.2;
        }
        else if (gamepad1.left_stick_y > 0.1 && gamepad1.left_stick_x < 0.3) {
            fLPower = gamepad1.left_stick_y;
            fRPower = -gamepad1.left_stick_y;
            rLPower = gamepad1.left_stick_y;
            rRPower = -gamepad1.left_stick_y;
        }
        else if (gamepad1.left_stick_x > 0.1 && gamepad1.left_stick_y < 0.3) {
            fLPower = gamepad1.left_stick_y;
            fRPower = gamepad1.left_stick_y;
            rLPower = -gamepad1.left_stick_y;
            rRPower = -gamepad1.left_stick_y;
        }
        else if (gamepad1.left_stick_x > 0.2 && gamepad1.left_stick_y > 0.2) {
            fLPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            fRPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            rLPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
            rRPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        }
        else {
            fLPower = 0;
            fRPower = 0;
            rLPower = 0;
            rRPower = 0;
        }

        fLPower -= gamepad1.right_stick_x;
        fRPower -= gamepad1.right_stick_x;
        rLPower -= gamepad1.right_stick_x;
        rRPower -= gamepad1.right_stick_x;

        if(gamepad2.x && !gamepad2.y)
        {
            fullSpeed = !fullSpeed;
        }

        if( gamepad2.x && gamepad2.y )
        {
            powerFactor = Math.abs(gamepad2.right_stick_x);
        }

        if (fullSpeed !=  true) {
            fLPower = 0.5 * fLPower;
            fRPower = 0.5 * fRPower;
            rLPower = 0.5 * rLPower;
            rRPower = 0.5 * rRPower;
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
        //telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Full Speed Enabled: ", fullSpeed);
        telemetry.addData("Speed Factor: ", powerFactor);
        telemetry.update();
    }
}