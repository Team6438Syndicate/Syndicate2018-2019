package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 *  https://docs.google.com/spreadsheets/d/13nvS4GjjWdywcON7hg1URirm_IHWoOywUKIuTGDfaUs/edit
 *  ^^
 *  Spreadsheet to check our hardware
 **/
public class Team6438HardwareMap
{
    /* public OpMode members. */
    //Motor mapping
    public DcMotor leftMotor        = null;
    public DcMotor rightMotor       = null;
    public DcMotor linearSlide      = null;
    public DcMotor intakeSpinner    = null;
    public DcMotor intakeMover      = null;

    //Servo mapping
    //public Servo colorSensorServo   = null;
    public CRServo teamMarkerServo = null;

    //Sensor Mapping
    public BNO055IMU imu            = null;
    //public ColorSensor colorSensor  = null;
    //public DistanceSensor distanceSensor = null;

    //Variables
    public  final double CPR = 288;

    //Drive Gear Reduction - This is < 1.0 if geared UP
    public  final double DGR = 1.0;

    //Wheel Diameter Inches
    public  final double WDI = 3.8;

    //Counts per Inch
    public  final double CPI = (CPR * DGR)
            / (WDI * Math.PI);

    //CPR for linear and intake motors
    public  final double otherCPR = 1120;

    //Gear Reduction for linear slide
    public  final double GRL = 2.0;

    // CPI for linear slide
    public  final double linearCPI = otherCPR * GRL;

    //Boolean to store teleOp Value
    public  boolean teleOp = true;

    //torquenado cpr
    public  final double torquenadoCPR = 1440;

    //intake gear ratio
    public  final double intakeGearRatio = 3.0;

    //intake cpi calc
    public  final double intakeCPI = torquenadoCPR * intakeGearRatio;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    //public ElapsedTime period = new ElapsedTime();

    //Vuforia variables
    public  final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public  final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public  final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public  final String VUFORIA_KEY = "ATEEWHn/////AAABmXzvuqxXZkYkr3AeTQT4Qg0P3tudpoBP/Rp2Xyw3zNlZYk+ZI5Jp/yo8TDf62o+UjdBvoe0LP5nNDqFESCtSImOG2WRuMkoESAyhSVzMU0hY53dWb4l0s7mCe+xqqT8i0r9pPdav7N7RiGHG7WYoIBXrQeyz+NEq8TLYTTCXmZMFgPeEU30Nb+t4JikoNMr0X0Ej6y1vG+7EX3O9KI8RXoPYbBmPzvX5uVvWBNg2J0g0SBiZUXa8pQOCxi0QyHyNUiwvV5WKnM2jncg+eI7im5s+k4yn6Xjaeecg6q9IT45YNvbhV4PM/LbwGQTKBf0AOCM/qL7tz7evypWw5uK15BayqAitBLy7Sr0SvIjYMjPg";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /* Constructor */
    public Team6438HardwareMap()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.get(DcMotor.class, "leftDrive");
        rightMotor = hwMap.get(DcMotor.class, "rightDrive");
        linearSlide = hwMap.get(DcMotor.class, "linearSlide");
        intakeSpinner = hwMap.get(DcMotor.class, "intakeSpinner");
        intakeMover = hwMap.get(DcMotor.class, "intakeMover");

        //Reverse the right Motor
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMover.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        linearSlide.setPower(0);
        intakeSpinner.setPower(0);
        intakeMover.setPower(0);

        //------------------------------------------------------------------------------------------

        // Define and initialize ALL installed servos.
        //colorSensorServo = hwMap.get(Servo.class, "colorSensorServo");
        //Select a start position
        //colorSensorServo.setPosition();
        teamMarkerServo = hwMap.get(CRServo.class, "teamMarkerServo");

        //------------------------------------------------------------------------------------------

        //Color sensor hardware map
        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        //Distance sensor hardware map
        //distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        //imu map
        imu = hwMap.get(BNO055IMU.class, "imu");
    }
}


