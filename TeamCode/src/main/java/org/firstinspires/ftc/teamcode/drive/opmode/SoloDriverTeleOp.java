package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp

public class SoloDriverTeleOp extends LinearOpMode {
    //
    DcMotor stangafata, dreaptafata, stangaspate, dreaptaspate;

    Servo cutie, control, retragere1, retragere2;
    int state =0;
    DcMotor intake1, intake2, extindere, carusel;
    DistanceSensor distanta;

    @Override
    public void runOpMode() {

        hardware part = new hardware(hardwareMap);
        cutie = part.getCutie();
        control = part.getControl();
        retragere1 = part.getRetragere1();
        retragere2=part.getRetragere2();
        state=0;
        //sasiu
        stangafata = part.getMotorstangafata();
        dreaptafata = part.getMotordreaptafata();
        stangaspate = part.getMotorstangaspate();
        dreaptaspate = part.getMotordreaptaspate();

        //motoare dc
        intake1 = part.getIntake1();
        intake2 = part.getIntake2();
        extindere = part.getExtindere();
        carusel = part.getCarusel();

        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);

        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retragere2.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            cutie.setPosition(0.43);
            //gamepad1 - miscare
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if (!gamepad1.left_bumper) {
                x /= 2;
                y /= 2;
            }
            if (!gamepad1.right_bumper) {
                turn /= 2;
            }
            mecanum(x, y, turn);

            //gamepad 2 - control brat + intake + carusel
            if (gamepad1.start)
                carusel.setPower(-0.80);
            else
                carusel.setPower(0);


            //controale intake
            if (gamepad1.right_trigger > 0) {
                intake1.setPower(-gamepad1.right_trigger);
                intake2.setPower(gamepad1.right_trigger);
                //in
            } else if (gamepad1.left_trigger > 0) {
                intake1.setPower(gamepad1.left_trigger);
                intake2.setPower(-gamepad1.left_trigger);
                //out
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            telemetry.addData("retragere1",retragere1.getPosition());
            telemetry.addData("retragere2",retragere2.getPosition());

            telemetry.addData("culisare",extindere.getCurrentPosition());
            telemetry.update();

            //urcare peste obstacole
            if (gamepad1.dpad_left) {
                retragere1.setPosition(-0.7);
                retragere2.setPosition(-0.7);
                //intake sus
                extindere.setTargetPosition(2500);
                extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extindere.setPower(0.6);
                //brat cutie sus
                control.setPosition(0.2);
            }

            //coborare
            if (gamepad1.dpad_down && extindere.getCurrentPosition()>2400) {
                retragere1.setPosition(0.55);
                retragere2.setPosition(0.55);
                //intake jos
                sleep(500);
                coborare();
                //brat cutie jos
                control.setPosition(0.54);
            }

            if (gamepad1.dpad_up)
                urcare();

            if (gamepad1.y) {
                control.setPosition(-1);
                //spate full
            }


        }

    }

    public void urcare(){
        extindere.setTargetPosition(0);
        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extindere.setPower(0.6);
    }
    public void coborare(){
        extindere.setTargetPosition(3250);
        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extindere.setPower(0.8);
        // control.setPosition(0.3);
    }

    public void jos_manual(){
        extindere.setTargetPosition(3300);
        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extindere.setPower(0.8);
    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        stangafata.setPower(v1);
        dreaptafata.setPower(v2);
        stangaspate.setPower(v3);
        dreaptaspate.setPower(v4);

    }
}

