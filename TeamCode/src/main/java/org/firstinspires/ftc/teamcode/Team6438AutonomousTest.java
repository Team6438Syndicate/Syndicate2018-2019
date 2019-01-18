package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="Autonomous Test", group = "Team 6438")
public class Team6438AutonomousTest extends LinearOpMode
{
    Team6438HardwareMap robot = new Team6438HardwareMap();
    //Cpi map
    double CPI = (int) robot.CPI;
boolean firstTime=true;
boolean secondTime=false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        if(firstTime==false)
        {
            secondTime=true;
        }
        robot.init(hardwareMap);
        waitForStart();
       
        while(opModeIsActive())
        {
          
            if(firstTime==true)
            {
                
                
                //move forward before turn
                encoderDrive(0.5,-10,-10);
                //turn towards depot
                encoderDrive(0.5,-47,47);
                //head into depot
                encoderDrive(.6, 45, 45);
                //turn out of depot and towards crater
                encoderDrive(.5,14,-14);
                //drive towards crater
                encoderDrive(.5,92,92);
            
            }
        }
    }
    //Initial motion for coming off of the lander
    private void releaseRobot(double speed, int servoDistance )
    {
        //linear actuator goes here
        
    }

    //Encoder drive method to drive the motors
    private void encoderDrive(double speed, double leftInches, double rightInches)
    {
        //Declaring new targets
        int leftTarget, rightTarget;

        //Gets the motors starting positions
        int startLeftPosition  = robot.leftMotor.getCurrentPosition();
        int startRightPosition = robot.rightMotor.getCurrentPosition();

        //Telemetry to show start position
        telemetry.addData("Left Start Position",startLeftPosition);
        telemetry.addData("Right Start Position", startRightPosition);
        telemetry.update();

        //Ensure we are in op mode
        if ( opModeIsActive() )
        {
            //Using the current position and the new desired position send this to the motor
            leftTarget  = startLeftPosition  + (int) (leftInches * CPI);
            rightTarget = startRightPosition + (int) (rightInches * CPI);
            robot.leftMotor.setTargetPosition(leftTarget);
            robot.rightMotor.setTargetPosition(rightTarget);

            //Turns the motors to run to position mode
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the power to the absolute value of the speed of the method input
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            //While opMode is still active and the motors are going add telemetry to tell the user where its going
            while(opModeIsActive() && robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
            {
                telemetry.addData("Running to ", " %7d :%7d", leftTarget, rightTarget);
                telemetry.addData("Currently At", " %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            //When done stop all the motion and turn off run to position
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            firstTime=false;
            secondTime=true;
        }
    }
}
