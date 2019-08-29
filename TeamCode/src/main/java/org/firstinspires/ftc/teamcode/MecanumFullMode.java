package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "Mecanum Full", group = "TeleOp 6438")
public class MecanumFullMode extends OpMode
{
    //Reference to our hardware map
    private Team6438HardwareMap robot = new Team6438HardwareMap();

    //Allows us a pattern variable for LED control
    RevBlinkinLedDriver.BlinkinPattern pattern;

    //booleans to control certain aspects of our TeleOp
    private boolean fullSpeed = false;      //full speed
    private boolean swap = false;           //overrides??
    private boolean go = true;              //??
    private DistanceSensor sensorRange;     //reference to our distance sensor

    private double powerFactor = 1;         //factor by which the motor power is multiplied
    private final int INTAKE_ROTATION_FULL = 600;

    //????
    int linearSlidePosition = 0;
    double linearSlidePower = 0;

    @Override
    public void init()
    {
        //init the hardware
        robot.init(hardwareMap);

        //Drive Motors should drive without encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pinionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Brake
        robot.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intakeMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeMover.setTargetPosition(0);

        //Pattern during init
        pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        robot.blinkinLedDriver.setPattern(pattern);         //set the pattern

        //telemetry
        telemetry.addData("Hardware Status: ", "Mapped");
        telemetry.update();
    }


    //read this https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html
    @Override
    public void loop()
    {
        //Variables for power
        double fLPower, fRPower, rLPower, rRPower;
        double pinionPower = 0, spinPower = 0;
        //Control for intake mineral grabbing and scoring
        if (gamepad2.a)
        {
            robot.intakeRotator.setPosition(1);
            intakeMove(1, INTAKE_ROTATION_FULL);
        }
        else if (gamepad2.x)
        {
            robot.intakeRotator.setPosition(.57);
            intakeMove(.1, 100);
        }
        else if(gamepad2.b)
        {
            robot.intakeRotator.setPosition(.51);
        }
        else if (gamepad2.y)
        {
            robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (gamepad2.left_trigger > .05)     //untested
        {
            intakeMove(1, robot.intakeMover.getCurrentPosition() + 50);
        }
        else if (gamepad2.right_trigger > .05)  //untested
        {
            intakeMove(1, robot.intakeMover.getCurrentPosition() - 50);
        }

        //intakeMove(1, robot.intakeMover.getCurrentPosition());

        //intake slide
        if (gamepad2.right_bumper)
        {
            moveIntakeSlide(1, -1050);
        }
        else if (gamepad2.left_bumper)
        {
            moveIntakeSlide(1, -100);
        }
        else if ( gamepad2.dpad_up && robot.intakeSlide.getCurrentPosition() < 1170)
        {
            moveIntakeSlide(1,robot.intakeSlide.getCurrentPosition() - 100);        //make the slide go further
        }
        else if ( gamepad2.dpad_down)
        {
            moveIntakeSlide(1,robot.intakeSlide.getCurrentPosition() + 100);
        }

        //rotates the servo
        if (gamepad2.dpad_left)
        {
            robot.intakeRotator.setPosition(robot.intakeRotator.getPosition() + 0.01);
        }
        else if (gamepad2.dpad_right)
        {
            robot.intakeRotator.setPosition(robot.intakeRotator.getPosition() - 0.01);
        }

        //moveIntakeSlide(1, robot.intakeSlide.getCurrentPosition());

        //Power variables for the stuff
        if ((gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) ) {
            pinionPower = gamepad2.left_stick_y;
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT;
            robot.blinkinLedDriver.setPattern(pattern);
        }

        if ((gamepad2.right_stick_y > 0.1) || gamepad2.right_stick_y < -0.1) {
            spinPower = gamepad2.right_stick_y;
            pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            robot.blinkinLedDriver.setPattern(pattern);
        }

        //Controls for tank treads
        if ((gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) && (gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3)) //forward
        {
            fLPower = -gamepad1.left_stick_y;
            fRPower = -gamepad1.left_stick_y;
            rLPower = -gamepad1.left_stick_y;
            rRPower = -gamepad1.left_stick_y;
            pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
            robot.blinkinLedDriver.setPattern(pattern);
        } else if ((gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_x < -0.1) && (gamepad1.left_stick_y < 0.3 && gamepad1.left_stick_y > -0.3)) //strafe
        {
            fLPower = gamepad1.left_stick_x;
            fRPower = -gamepad1.left_stick_x;
            rLPower = -gamepad1.left_stick_x;
            rRPower = gamepad1.left_stick_x;
            pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
            robot.blinkinLedDriver.setPattern(pattern);
        }
        //what is this
        else if ((gamepad1.left_stick_x > 0.3 && gamepad1.left_stick_y > 0.3) || (gamepad1.left_stick_x < -0.3 && gamepad1.left_stick_y > 0.3) || (gamepad1.left_stick_x > 0.3 && gamepad1.left_stick_y < -0.3) || (gamepad1.left_stick_x < -0.3 && gamepad1.left_stick_y < -0.3))//diagonal
        {
            fLPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            fRPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            rLPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            rRPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
            robot.blinkinLedDriver.setPattern(pattern);
        }
        else {
            fLPower = 0;
            fRPower = 0;
            rLPower = 0;
            rRPower = 0;
        }

        //why
        /**
         * Here is a problem
         * will continually grow
         */
        fLPower += gamepad1.right_stick_x;
        fRPower -= gamepad1.right_stick_x;
        rLPower += gamepad1.right_stick_x;
        rRPower -= gamepad1.right_stick_x;
        //pattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY;
        //robot.blinkinLedDriver.setPattern(pattern);



        if (gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= .01) {
            fLPower = 0.5 * fLPower;
            fRPower = 0.5 * fRPower;
            rLPower = 0.5 * rLPower;
            rRPower = 0.5 * rRPower;
        }
        else
        {
            fLPower = 0.75 * fLPower;
            fRPower = 0.75 * fRPower;
            rLPower = 0.75 * rLPower;
            rRPower = 0.75 * rRPower;

        }

        //apply the power
        robot.leftFrontMotor.setPower(fLPower);
        robot.rightFrontMotor.setPower(fRPower);
        robot.leftRearMotor.setPower(rLPower);
        robot.rightRearMotor.setPower(rRPower);
        robot.pinionLift.setPower(pinionPower);
        robot.leftIntake.setPower(spinPower);
        robot.rightIntake.setPower(spinPower);

        //Telemetry to constantly refresh data to update user
        telemetry.addData("Front Left Power: ", fLPower);
        telemetry.addData("Front Right Power: ", fRPower);
        telemetry.addData("Rear Left Power: ", rLPower);
        telemetry.addData("Rear Right Power: ", rRPower);
        telemetry.addData("Pinion Power ", pinionPower);
        telemetry.addData("Intake Spin ", spinPower);
        telemetry.addData("Intake Position", robot.intakeMover.getCurrentPosition());
        telemetry.addData("Intake Power", robot.intakeMover.getPower());
        telemetry.addData("Servo Head Rotation ", robot.intakeRotator.getPosition());
        telemetry.addData("Encoder For Pinion ", robot.pinionLift.getCurrentPosition());
        telemetry.addData("Speed Factor: ", powerFactor);
        telemetry.addData("Swap? ", swap);
        telemetry.update();
    }

    //Method to move the intake slide
    private void moveIntakeSlide(double speed, int position)
    {
        //Set the target
        robot.intakeSlide.setTargetPosition(position);

        //Turn On RUN_TO_POSITION
        robot.intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start motion
        robot.intakeSlide.setPower(speed);

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);

        while (robot.intakeSlide.isBusy())
        {
            pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
            robot.blinkinLedDriver.setPattern(pattern);
            telemetry.addData("Moving to", robot.intakeSlide.getTargetPosition());
            telemetry.addData("Currently At", robot.intakeSlide.getCurrentPosition());
            telemetry.update();

        }
    }

    //Method to move the intake
    private void intakeMove(double speed, int position)
    {
        //Set the target
        robot.intakeMover.setTargetPosition(position);

        //Turn On RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //start motion.
        robot.intakeMover.setPower(Math.abs(speed));

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);

        while (robot.intakeMover.isBusy())
        {
            pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
            robot.blinkinLedDriver.setPattern(pattern);
            telemetry.addData("Currently at ", robot.intakeMover.getCurrentPosition());
            telemetry.addData("Going to ", robot.intakeMover.getTargetPosition());
            telemetry.update();

        }
    }
}
