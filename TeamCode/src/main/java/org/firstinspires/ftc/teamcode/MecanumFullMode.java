package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "Mecanum Full", group = "TeleOp 6438")
public class MecanumFullMode extends OpMode {
    //Reference to our hardware map
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    private boolean fullSpeed = false;
    private DistanceSensor sensorRange;
    //DcMotor intakeSlide;
    //DcMotor intakeMover;
    //DcMotor linearActuator;
    private double powerFactor = 1;
    private final int INTAKE_ROTATION_FULL = 3300;

    int linearSlidePosition = 0;
    double linearSlidePower = 0;

    @Override
    public void init() {
        //init the hardware
        robot.init(hardwareMap);

        //Map motors
    /*  robot.leftFrontMotor        = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        robot.rightFrontMotor       = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        robot.leftRearMotor         = hardwareMap.get(DcMotor.class, "leftRearDrive");
        robot.rightRearMotor        = hardwareMap.get(DcMotor.class, "rightRearDrive");
        linearActuator              = hardwareMap.get(DcMotor.class, "linearActuator");
        intakeSlide                 = hardwareMap.get(DcMotor.class, "intakeSlide");
        intakeMover                 = hardwareMap.get(DcMotor.class, "intakeMover");

        //Map servos
        intakeSpin            = hardwareMap.get(Servo.class, "intakeSpin");

        intakeSpin = hardwareMap.get(Servo.class, "cameraMount");

        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        */

        //Drive Motors should drive without encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pinionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intakeMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //telemetry
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();


    }


    //read this https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html
    @Override
    public void loop() {
        //Variables for power
        double fLPower, fRPower, rLPower, rRPower, actuatorPower;

        //more test
        //linearSlidePower = gamepad2.right_stick_y;
        //intakeSlide.setPower(linearSlidePower);

        //Control for intake mineral grabbing and scoring
        if (gamepad2.a) {
            intakeUp(1);
        } else if (gamepad2.b) {
            intakeDown(1);
        } else if (gamepad2.right_trigger > .05)     //untested
        {
            intakeMove(0.6, robot.intakeMover.getCurrentPosition() - 250);
        } else if (gamepad2.left_trigger > .05)  //untested
        {
            intakeMove(1, robot.intakeMover.getCurrentPosition() + 250);
        }
/*   David: Commented out because no clear purpose. Is this for automatic correction? Brad pls explain.
        if (robot.intakeMover.getCurrentPosition() > 2200)
        {
            robot.intakeMover.setPower(1);
            robot.intakeMover.setTargetPosition(robot.intakeMover.getCurrentPosition() - 1);
        }
        else
        {
            robot.intakeMover.setPower(1);
            robot.intakeMover.setTargetPosition(robot.intakeMover.getCurrentPosition() + 1);
        }
*/
        //Controls for intake arm
        if (gamepad2.right_bumper) {
            moveIntakeSlide(1, 1500);
        } else if (gamepad2.left_bumper) {
            moveIntakeSlide(1, 0);
        }

        robot.intakeSlide.setPower(1);
        robot.intakeSlide.setTargetPosition(robot.intakeSlide.getCurrentPosition() + 1);

        actuatorPower = gamepad2.left_stick_y;

        //Controls for tank treads
        if ((gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) && (gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3)) {
            fLPower = -gamepad1.left_stick_y;
            fRPower = gamepad1.left_stick_y;
            rLPower = -gamepad1.left_stick_y;
            rRPower = gamepad1.left_stick_y;
        } else if ((gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_x < -0.1) && (gamepad1.left_stick_y < 0.3 && gamepad1.left_stick_y > -0.3)) {
            fLPower = gamepad1.left_stick_x;
            fRPower = gamepad1.left_stick_x;
            rLPower = -gamepad1.left_stick_x;
            rRPower = -gamepad1.left_stick_x;
        } else if ((gamepad1.left_stick_x > 0.3 && gamepad1.left_stick_y > 0.3) || (gamepad1.left_stick_x < -0.3 && gamepad1.left_stick_y > 0.3) || (gamepad1.left_stick_x > 0.3 && gamepad1.left_stick_y < -0.3) || (gamepad1.left_stick_x < -0.3 && gamepad1.left_stick_y < -0.3)) {
            fLPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            fRPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            rLPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            rRPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
        } else {
            fLPower = 0;
            fRPower = 0;
            rLPower = 0;
            rRPower = 0;
        }

        fLPower -= gamepad1.right_stick_x;
        fRPower -= gamepad1.right_stick_x;
        rLPower -= gamepad1.right_stick_x;
        rRPower -= gamepad1.right_stick_x;

        if (gamepad2.x && !gamepad2.y) {
            fullSpeed = !fullSpeed;
        }

        if (gamepad2.x && gamepad2.y) {
            powerFactor = Math.abs(gamepad2.right_stick_y);
        }

        if (!fullSpeed) {
            fLPower = 0.5 * fLPower;
            fRPower = 0.5 * fRPower;
            rLPower = 0.5 * rLPower;
            rRPower = 0.5 * rRPower;
        }

        robot.leftFrontMotor.setPower(fLPower);
        robot.rightFrontMotor.setPower(fRPower);
        robot.leftRearMotor.setPower(rLPower);
        robot.rightRearMotor.setPower(rRPower);
        //linearActuator.setPower(actuatorPower);

        //Telemetry to constantly refresh data to update user
        telemetry.addData("Front Left Power: ", fLPower);
        telemetry.addData("Front Right Power: ", fRPower);
        telemetry.addData("Rear Left Power: ", rLPower);
        telemetry.addData("Rear Right Power: ", rRPower);
        telemetry.addData("Actuator Power", actuatorPower);
        telemetry.addData("Intake Height", robot.intakeMover.getCurrentPosition());
        telemetry.addData("Intake Speed", robot.intakeMover.getPower());
        //telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Full Speed Enabled: ", fullSpeed);
        telemetry.addData("Speed Factor: ", powerFactor);
        //telemetry.addData("Linear Position:", linearSlidePosition);
        telemetry.update();
    }

    //Method to move the intake slide
    private void moveIntakeSlide(double speed, int position) {
        //Set the target
        robot.intakeSlide.setTargetPosition(position);

        //Turn On RUN_TO_POSITION
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start motion
        robot.intakeSlide.setPower(speed);

        while (robot.intakeSlide.isBusy()) {
            telemetry.addData("Moving to", robot.intakeSlide.getTargetPosition());
            telemetry.addData("Currently At", robot.intakeSlide.getCurrentPosition());
            telemetry.update();
        }
    }

    //Method to move the intake
    private void intakeMove(double speed, int position) {
        //Set the target
        robot.intakeMover.setTargetPosition(position);

        // Turn On RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        robot.intakeMover.setPower(Math.abs(speed));

        while (robot.intakeMover.isBusy()) {
            telemetry.addData("Currently at ", robot.intakeMover.getTargetPosition());
            telemetry.addData("Going to ", robot.intakeMover.getCurrentPosition());
            telemetry.update();
        }
    }

    private void intakeUp(double speed) {
        double spinPosition = 1;
        //Set the arm's target
        robot.intakeMover.setTargetPosition(INTAKE_ROTATION_FULL);

        // Turn On RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start arm motion.
        robot.intakeMover.setPower(Math.abs(speed));

        while (robot.intakeMover.isBusy()) {
            telemetry.addData("Arm Currently at ", robot.intakeMover.getCurrentPosition());
            telemetry.addData("Arm Going to ", robot.intakeMover.getTargetPosition());
            //telemetry.addData("Head Currently at ", robot.intakeSpin.getPosition());
            telemetry.addData("Head Going to ", spinPosition);
            telemetry.update();
        }
    }

    private void intakeDown(double speed) {
        double spinPosition = 0;
        //Set the arm's target
        robot.intakeMover.setTargetPosition(0);

        // Turn On RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start arm motion.
        robot.intakeMover.setPower(Math.abs(speed));

        while (robot.intakeMover.isBusy()) {
            telemetry.addData("Arm Currently at ", robot.intakeMover.getCurrentPosition());
            telemetry.addData("Arm Going to ", robot.intakeMover.getTargetPosition());
            //telemetry.addData("Head Currently at ", robot.intakeSpin.getPosition());
            telemetry.addData("Head Going to ", spinPosition);
            telemetry.update();
        }
    }
}