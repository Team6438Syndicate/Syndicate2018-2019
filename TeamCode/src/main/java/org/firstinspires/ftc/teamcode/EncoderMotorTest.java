package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test",group ="test")
public class EncoderMotorTest extends OpMode
{
    //Declaring Motors
    private DcMotor a,b,c,d;

    @Override
    public void init()
    {
        a = hardwareMap.dcMotor.get("a");
        b = hardwareMap.dcMotor.get("b");
        c = hardwareMap.dcMotor.get("c");
        d = hardwareMap.dcMotor.get("d");

        a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        c.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mapped ", "True");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        if (gamepad1.a)
        {
            a.setPower(1);
            b.setPower(1);
            c.setPower(1);
            d.setPower(1);
        }

        if(gamepad1.b)
        {
            a.setPower(0);
            b.setPower(0);
            c.setPower(0);
            d.setPower(0);
        }

            telemetry.addData("Press A to spin the motors", "");
            telemetry.addData("And B to stop them","");
            telemetry.addData("Direction of motion doesnt matter","");
            telemetry.addData("So long as encoder and motor move in same dir","");
            telemetry.addData("If encoder reads 0,1 for prolonged time","");
            telemetry.addData("its broken","");
            telemetry.addData("a",a.getCurrentPosition());
            telemetry.addData("b",b.getCurrentPosition());
            telemetry.addData("c",c.getCurrentPosition());
            telemetry.addData("d",d.getCurrentPosition());
            telemetry.update();
    }
}
