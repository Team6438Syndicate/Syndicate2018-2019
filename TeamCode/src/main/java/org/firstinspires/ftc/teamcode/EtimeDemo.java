package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class EtimeDemo extends LinearOpMode
{
    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void runOpMode() throws InterruptedException
    {
        //init the robot
        robot.init(hardwareMap);

        //create a new elapsed time instance
        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //telemetry to let the user know the opmode is ready
        telemetry.addData("Ready to Start :", "TRUE");
        telemetry.update();

        //wait for the start button to be pressed
        waitForStart();

        //loop while the op mode is active
        while(opModeIsActive())
        {
            telemetry.addData("time right now = ", etime.now(TimeUnit.MILLISECONDS));
            telemetry.update();
        }
    }
}
