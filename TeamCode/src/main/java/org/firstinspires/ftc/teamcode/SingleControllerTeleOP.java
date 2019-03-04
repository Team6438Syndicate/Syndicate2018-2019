package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SingleControllerTeleOP extends OpMode
{

    Team6438HardwareMap robot = new Team6438HardwareMap();

    @Override
    public void init()
    {
        //Init the hardware
        robot.init(hardwareMap);

        //Reset encoders
        robot.intakeMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Drive Motors should drive without encoders
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Add stuff for the actuator and




    }

    @Override
    public void start()
    {
        robot.cameraMount.setPosition(robot.cameraMountTucked);

    }

    @Override
    public void loop()
    {

    }
}
