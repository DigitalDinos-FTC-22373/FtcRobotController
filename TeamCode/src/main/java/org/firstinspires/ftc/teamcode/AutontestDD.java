/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE

 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "My Autonomous", group = "Competition")
//@Disabled
public class AutontestDD extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake;
    private DcMotor backintake;
    private CRServo feederleft;
    private CRServo feederright;
    private DcMotorEx shooter;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back right");

        intake = hardwareMap.get(DcMotor.class, "intake");
        backintake = hardwareMap.get(DcMotor.class,"backintake");
        //Vineeth made the change
        feederleft = hardwareMap.get(CRServo.class, "feeder left");
        feederright = hardwareMap.get(CRServo.class, "feeder right");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        backintake.setDirection(DcMotor.Direction.FORWARD);
        feederleft.setDirection(CRServo.Direction.FORWARD);
        feederright.setDirection(CRServo.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotor.RunMode.);
        double shooterPower = 0.0;
        shooter.setPower(shooterPower);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)


        if (opModeIsActive()) {

            //Demo Auton
            shooter.setPower(0.6);
            sleep(2500);
            feederleft.setPower(1);
            feederright.setPower(1);
            // shot 1
            sleep(3000);
            shooter.setPower(0.7);
            feederleft.setPower(0);
            feederright.setPower(0);
            sleep(1000);
            backintake.setPower(-0.4);
            intake.setPower(-0.4);
            sleep(1000);
            backintake.setPower(0);
            intake.setPower(0);
            feederleft.setPower(1);
            feederright.setPower(1);
            //shot 2
            sleep(5000);
            shooter.setPower(0.75);
            backintake.setPower(-0.4);
            feederleft.setPower(1);
            feederright.setPower(1);
            intake.setPower(-0.4);
            sleep(5000);

            setMovement(0.5,0,0);
            sleep(1500);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("shooter velocity", ((DcMotorEx) shooter).getVelocity());
        telemetry.addData("shooter power", ((DcMotorEx) shooter).getPower());
        telemetry.addData("675",10);
        telemetry.update();
    }
   // hello
//
    private void setMovement(double forward, double sideways, double turn) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = forward + sideways + turn;
        double frontRightPower = forward - sideways - turn;
        double backLeftPower   = forward - sideways + turn;
        double backRightPower  = forward + sideways - turn;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}