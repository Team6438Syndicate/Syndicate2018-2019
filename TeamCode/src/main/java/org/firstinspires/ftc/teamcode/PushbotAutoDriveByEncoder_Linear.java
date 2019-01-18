package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Team6438HardwareMap robot = new Team6438HardwareMap();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //Stop and reset encoders
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Run using encoders
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0 Starting at",robot.intakeMover.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(1,  800, 25000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double position, long hold)
    {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.intakeMover.getCurrentPosition() + (int)(position);

            //Set the target
            robot.intakeMover.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.intakeMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            robot.intakeMover.setPower(Math.abs(speed));

            while (opModeIsActive() && robot.intakeMover.isBusy() )
            {
                // Display it for the driver.
                telemetry.addData("Path1: Running to", robot.intakeMover.getTargetPosition()  );
                telemetry.addData("Path2: Running at", robot.intakeMover.getCurrentPosition() );
                telemetry.update();
            }

            // Stop all motion;
            //robot.intakeMover.setPower(0);

            // Turn off RUN_TO_POSITION
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //sleep(hold);   // optional pause after each move
        }
    }
}
