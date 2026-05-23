

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

    /**
     * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
     * This assumes that you are using a REV Robotics Control Hub or REV Robotics Expansion Hub
     * as your DC motor controller.  This OpMode uses the extended/enhanced
     * PIDF-related functions of the DcMotorEx class.
     */

    @Config
    @TeleOp
    public class Flywheel_tuning extends LinearOpMode {

        // our DC motor
        DcMotorEx shooter;

        public double NEW_P = 150;
        public static final double NEW_I = 0;
        public static final double NEW_D = 0;
        public double NEW_F = 13.6;
        // These values are for illustration only; they must be set
        // and adjusted for each motor based on its planned usage.
        public double set_velocity = 1200;

        public void runOpMode() {
            // Get reference to DC motor.
            // Since we are using the Control Hub or Expansion Hub,
            // cast this motor to a DcMotorEx object.
            shooter = (DcMotorEx)hardwareMap.get(DcMotor.class, "shooter");
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();

            // wait for start command
            waitForStart();
            shooter.setDirection(DcMotor.Direction.REVERSE);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
            PIDFCoefficients pidfOrig = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            // Change coefficients using methods included with DcMotorEx class.
            PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            // Re-read coefficients and verify change.
            PIDFCoefficients pidfModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocity(set_velocity);

            // display info to user
            while(opModeIsActive()) {

                if (gamepad1.right_bumper){
                    NEW_P += 0.1;
                }
                if (gamepad1.left_bumper){
                    NEW_P -= 0.1;
                }

                if (gamepad1.aWasPressed()){
                    if (set_velocity > 1000) {
                        set_velocity = 800;
                    }
                    else if (set_velocity < 1000){
                        set_velocity = 1200;
                    }
                }

                pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
                shooter.setVelocity(set_velocity);
                pidfModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
                telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                        pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
                telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                        pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
                telemetry.addData("shooter get velocity",  shooter.getVelocity());
                telemetry.addData("set_velocity",  set_velocity);
                telemetry.update();
            }
        }
    }

