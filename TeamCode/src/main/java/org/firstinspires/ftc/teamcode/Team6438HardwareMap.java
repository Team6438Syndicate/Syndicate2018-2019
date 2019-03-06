/**
 * Name: Team6438HardwareMap
 * Purpose: This class contains maps for all hardware on the robot
 *          To reference it you need to create an instance of the Team6438HardwareMap class
 * Author: Matthew Batkiewicz
 * Contributors: Bradley Abelman, Matthew Kaboolian
 * Creation: 11/8/18
 * Last Edit: 1/20/19
 * Additional Notes: https://docs.google.com/spreadsheets/d/13nvS4GjjWdywcON7hg1URirm_IHWoOywUKIuTGDfaUs/edit
 *                   ^^Spreadsheet to check our hardware
 **/
package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

class Team6438HardwareMap
{


    //Motor mapping
    DcMotor leftMotor        = null;
    DcMotor rightMotor       = null;
    DcMotor linearActuator   = null;
    DcMotor intakeMover      = null;
    DcMotor intakeSlide      = null;

    //Servo mapping
    Servo teamMarkerServo   = null;
    Servo cameraMount = null;
    CRServo leftIntake = null;
    CRServo rightIntake = null;

    //Sensor Mapping
    BNO055IMU imu     = null;

    //Variables
    private final double hexCPR = 288;

    //Drive Gear Reduction - This is < 1.0 if geared UP
    private final double DGR = 1.0;

    //Wheel Diameter Inches
    private final double WDI = 3.8;

    //Counts per Inch
    final double hexCPI = (hexCPR * DGR)
                        / (WDI * Math.PI);


    //Camera mount servo positions
    final double cameraMountTucked = 1;
    final double cameraMountCenter = .15;
    final double cameraMountRight = 0;

    //Positions for intake
    final int intakeOutPosition = 2082;                  //This is the position just high enough to clear the crater
    final int intakeDunk = 1450;                         //This is the position where the balls fall into the lander
    final int intakeFloor = 2200;                        //This is the position where the intake is above the floor
    final int intakeMinimum = 0;                        //This is the value closest to the linear Actuator before causing problems
    final int intakeMax = 2400;                            //This is the lowest the intake can be before causing the motor to lock.

    //Positions for the slides
    final int slideExtended = 791;                        //This is the motor encoder position when the slide is all the way out
    final int slideUnExtended = 0;                      //This is the motor encoder position when the slide is all the way in

    //Variables for the actuator
    final int laInterference = 10000;                   //Min position where the linear actuator can be as to not cause damage

    //Vuforia variables
    final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    final String LABEL_SILVER_MINERAL = "Silver Mineral";
    final String VUFORIA_KEY = "ATEEWHn/////AAABmXzvuqxXZkYkr3AeTQT4Qg0P3tudpoBP/Rp2Xyw3zNlZYk+ZI5Jp/yo8TDf62o+UjdBvoe0LP5nNDqFESCtSImOG2WRuMkoESAyhSVzMU0hY53dWb4l0s7mCe+xqqT8i0r9pPdav7N7RiGHG7WYoIBXrQeyz+NEq8TLYTTCXmZMFgPeEU30Nb+t4JikoNMr0X0Ej6y1vG+7EX3O9KI8RXoPYbBmPzvX5uVvWBNg2J0g0SBiZUXa8pQOCxi0QyHyNUiwvV5WKnM2jncg+eI7im5s+k4yn6Xjaeecg6q9IT45YNvbhV4PM/LbwGQTKBf0AOCM/qL7tz7evypWw5uK15BayqAitBLy7Sr0SvIjYMjPg";
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    //Place in a number 0.0 - 1.0
    static final double confidence = .5;

    //Positions for the team marker servo
    final double toss = 1.0;
    final double tucked = 0.0;

    //Method to initialize standard Hardware interfaces
    void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        //local OpMode members.
        HardwareMap hwMap = ahwMap;

        //Define and Initialize Motors
        leftMotor        = hwMap.get(DcMotor.class, "leftDrive");
        rightMotor       = hwMap.get(DcMotor.class, "rightDrive");
        linearActuator   = hwMap.get(DcMotor.class, "linearActuator");
        intakeMover      = hwMap.get(DcMotor.class, "intakeMover");
        intakeSlide      = hwMap.get(DcMotor.class,"intakeSlide");

        //Reverse the right Motor and linear slide motor to make code operation easier
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        linearActuator.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMover.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);                               //Could change

        //Set all motors to zero power to prevent unintended movement
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        linearActuator.setPower(0);
        intakeMover.setPower(0);
        intakeSlide.setPower(0);

        //------------------------------------------------------------------------------------------
        // Define and initialize ALL installed servos.

        teamMarkerServo = hwMap.get(Servo.class, "teamMarkerServo");
        cameraMount = hwMap.get(Servo.class, "cameraMount");
        leftIntake = hwMap.get(CRServo.class,"leftIntake");
        rightIntake = hwMap.get(CRServo.class,"rightIntake");

        //Maybe??
        //rightIntake.setDirection(CRServo.Direction.REVERSE);

        //------------------------------------------------------------------------------------------
        // Define and initialize ALL installed sensors.
        imu = hwMap.get(BNO055IMU.class, "imu");
    }
}


