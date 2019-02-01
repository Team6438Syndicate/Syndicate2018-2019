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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

class Team6438HardwareMap
{
    //Motor mapping
     DcMotor leftMotor        = null;
     DcMotor rightMotor       = null;
     DcMotor linearActuator   = null;
     DcMotor intakeSpinner    = null;
     DcMotor intakeMover      = null;
     DcMotor intakeSlide     = null;

    //Servo mapping
    CRServo teamMarkerServo   = null;

    //Sensor Mapping
    private BNO055IMU imu     = null;

    //Variables
    private final double CPR = 288;

    //Drive Gear Reduction - This is < 1.0 if geared UP
    private final double DGR = 1.0;

    //Wheel Diameter Inches
    private final double WDI = 3.8;

    //Counts per Inch
    final double CPI = (CPR * DGR)
                     / (WDI * Math.PI);

    //CPR for linear and intake motors
    private final double otherCPR = 1120;

    //Gear Reduction for linear slide
    private final double GRL = 2.0;

    // CPI for linear slide
    final double linearCPI = otherCPR * GRL;

    //Vuforia variables
     final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
     final String LABEL_SILVER_MINERAL = "Silver Mineral";
     final String VUFORIA_KEY = "ATEEWHn/////AAABmXzvuqxXZkYkr3AeTQT4Qg0P3tudpoBP/Rp2Xyw3zNlZYk+ZI5Jp/yo8TDf62o+UjdBvoe0LP5nNDqFESCtSImOG2WRuMkoESAyhSVzMU0hY53dWb4l0s7mCe+xqqT8i0r9pPdav7N7RiGHG7WYoIBXrQeyz+NEq8TLYTTCXmZMFgPeEU30Nb+t4JikoNMr0X0Ej6y1vG+7EX3O9KI8RXoPYbBmPzvX5uVvWBNg2J0g0SBiZUXa8pQOCxi0QyHyNUiwvV5WKnM2jncg+eI7im5s+k4yn6Xjaeecg6q9IT45YNvbhV4PM/LbwGQTKBf0AOCM/qL7tz7evypWw5uK15BayqAitBLy7Sr0SvIjYMjPg";
     VuforiaLocalizer vuforia;
     TFObjectDetector tfod;
     //Place in a number 0.0 - 1.0
     public static final double confidence = .75;

    //Method to initialize standard Hardware interfaces
    void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        //local OpMode members.
        HardwareMap hwMap = ahwMap;

        //Define and Initialize Motors
        leftMotor     = hwMap.get(DcMotor.class, "leftDrive");
        rightMotor    = hwMap.get(DcMotor.class, "rightDrive");
        linearActuator   = hwMap.get(DcMotor.class, "linearActuator");         //Change this in the config
        intakeSpinner = hwMap.get(DcMotor.class, "intakeSpinner");
        intakeMover   = hwMap.get(DcMotor.class, "intakeMover");
        intakeSlide = hwMap.get(DcMotor.class,"intakeSlide");

        //Reverse the right Motor and linear slide motor to make code operation easier
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        linearActuator.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMover.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);                               //Could change

        //Set all motors to zero power to prevent unintended movement
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        linearActuator.setPower(0);
        intakeSpinner.setPower(0);
        intakeMover.setPower(0);
        intakeSlide.setPower(0);

        //------------------------------------------------------------------------------------------

        // Define and initialize ALL installed servos.
        teamMarkerServo = hwMap.get(CRServo.class, "teamMarkerServo");

        //------------------------------------------------------------------------------------------
        //imu map
        imu = hwMap.get(BNO055IMU.class, "imu");
    }
}


