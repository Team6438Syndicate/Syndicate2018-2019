package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "Mecanum Full w/ modded intake", group = "TeleOp 6438")
public class MecanumFullMode2 extends OpMode
{
    //Reference to our hardware map
    Team6438HardwareMap robot = new Team6438HardwareMap();

    //Boolean for the speed limiter
    private boolean fullSpeed = false;
    //private double powerFactor = 1;

    //Distance sensor
    private DistanceSensor sensorRange;

    //Boolean to determine if actuator is controlled by user or method
    private boolean actuatorControl = true;

    @Override
    public void init()
    {
        //init the hardware
        robot.init(hardwareMap);

        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        //Drive Motors should drive without encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Puts the motors on brake
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
        if(gamepad2.y) {
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftRearMotor.setPower(0);
            robot.rightRearMotor.setPower(0);
        } else loop1();
    }

    public void loop1()
    {
        //Variables for power
        double fLPower, fRPower, rLPower, rRPower, actuatorPower;
        double linearActuatorPower;

        //Controls for intake slide
        if (gamepad2.right_bumper)
        {
            moveIntakeSlide(1,robot.intakeSlide.getCurrentPosition() + 100);
        }
        else if (gamepad2.left_bumper)
        {
            moveIntakeSlide(1,robot.intakeSlide.getCurrentPosition() - 100);
        }

        //Control for intakeMover
        if(gamepad2.right_trigger > .05)     //untested
        {
            intakeMove(1,robot.intakeMoverLeft.getCurrentPosition() + 100);
        }
        else if (gamepad2.left_trigger > .05)  //untested
        {
            intakeMove ( 1, robot.intakeMoverLeft.getCurrentPosition() - 100);
        }

        //Controls for MECANUMS
        if ( (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) && (gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3) )
        {
            fLPower = -gamepad1.left_stick_y;
            fRPower = gamepad1.left_stick_y;
            rLPower = -gamepad1.left_stick_y;
            rRPower = gamepad1.left_stick_y;
        }
        else if ( (gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_x < -0.1) && (gamepad1.left_stick_y < 0.3 && gamepad1.left_stick_y > -0.3) )
        {
            fLPower = gamepad1.left_stick_x;
            fRPower = gamepad1.left_stick_x;
            rLPower = -gamepad1.left_stick_x;
            rRPower = -gamepad1.left_stick_x;
        }
        else if ( (gamepad1.left_stick_x > 0.3 && gamepad1.left_stick_y > 0.3) || (gamepad1.left_stick_x < -0.3 && gamepad1.left_stick_y > 0.3) || (gamepad1.left_stick_x > 0.3 && gamepad1.left_stick_y < -0.3) || (gamepad1.left_stick_x < -0.3 && gamepad1.left_stick_y < -0.3) )
        {
            fLPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            fRPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            rLPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            rRPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
        }
        else
            {
            fLPower = 0;
            fRPower = 0;
            rLPower = 0;
            rRPower = 0;
        }

        //sets the power for turns
        fLPower -= gamepad1.right_stick_x;
        fRPower -= gamepad1.right_stick_x;
        rLPower -= gamepad1.right_stick_x;
        rRPower -= gamepad1.right_stick_x;

        //Enables or disables fullspeed
        if(gamepad2.x) //&& !gamepad2.y)
        {
            fullSpeed = !fullSpeed;
        }

        //Not on right now
        /*
        if( gamepad2.x && gamepad2.y )

        {
            powerFactor = Math.abs(gamepad2.right_stick_y);
        }
        */


        if (!fullSpeed)
        {
            //fLPower *= 0.5;     //this is the more efficient way
            fLPower = 0.5 * fLPower;
            fRPower = 0.5 * fRPower;
            rLPower = 0.5 * rLPower;
            rRPower = 0.5 * rRPower;
        }

        if(actuatorControl)
        {
            linearActuatorPower = -gamepad2.right_stick_y;
            robot.linearActuator.setPower(linearActuatorPower);
        }
        else if (gamepad2.right_stick_button)
        {
            //retract the actuator
            moveLinearActuator(1,0);
        }

        //Applies the power to the motors
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
        //telemetry.addData("Speed Factor: ", powerFactor);
        //telemetry.addData("Linear Position:", linearSlidePosition);
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
        robot.intakeSlide.setPower(Math.abs(speed));

        while (robot.intakeSlide.isBusy())
        {
            telemetry.addData("Moving to", robot.intakeSlide.getTargetPosition());
            telemetry.addData("Currently At", robot.intakeSlide.getCurrentPosition());
            telemetry.update();
        }
    }

    //Method to move the intake
    private void intakeMove(double speed, int position)
    {
        //Set the target
        robot.intakeMoverLeft.setTargetPosition(position);
        robot.intakeMoverRight.setTargetPosition(position);

        // Turn On RUN_TO_POSITION
        robot.intakeMoverLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeMoverRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        robot.intakeMoverLeft.setPower(Math.abs(speed));
        robot.intakeMoverRight.setPower(Math.abs(speed));

        while ( ( robot.intakeMoverLeft.isBusy() || robot.intakeMoverRight.isBusy() ) ||  ( robot.intakeMoverLeft.isBusy() && robot.intakeMoverRight.isBusy() ) )
        {
            telemetry.addData("Left Motor Currently at: ", robot.intakeMoverLeft.getTargetPosition());
            telemetry.addData("Right Motor Currently at: ", robot.intakeMoverRight.getTargetPosition());
            telemetry.addData("Left Motor Going to: ", robot.intakeMoverLeft.getTargetPosition());
            telemetry.addData("Right Motor Going to: ", robot.intakeMoverRight.getTargetPosition());
            telemetry.update();
        }
    }

    //Method to move the linear actuator
    private void moveLinearActuator(double speed, int position)
    {
        //Makes it so the user no longer has control of linear actuator
        actuatorControl = false;

        //set the target
        robot.linearActuator.setTargetPosition(position);

        //run to position
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //apply the power
        robot.linearActuator.setPower(Math.abs(speed));

        while ( robot.linearActuator.isBusy() )
        {
            telemetry.addData("Linear Actuator Currently At: ", robot.linearActuator.getCurrentPosition());
            telemetry.addData("Linear Actuator: ", robot.linearActuator.getTargetPosition());
            telemetry.update();
        }
    }

    //method to find home and hang
    private void findHome(double speed)
    {

    }
    //End of opmode
}