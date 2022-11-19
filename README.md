/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop", group="Robot")
public class Teleop extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive;
        double turn;

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            float RFspeed = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            float RBspeed = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;

            double maxspeed = 0.5;

            if (gamepad1.right_bumper) {
                maxspeed = 1;
            }

            if (gamepad1.right_trigger != 0) {
                maxspeed = 0.15;
            }

            if (gamepad1.left_bumper) {
                robot.setSliderPower(0.1, 0.1);
            } else if (gamepad1.left_trigger != 0) {
                robot.setSliderPower(-0.2, -0.2);
            } else {
                robot.setSliderPower(0, 0);
            }
//            robot.setSliderPower(0.5,0.5);
//            sleep(1000);
//            robot.setSliderPower(-0.5,-0.5);
//            sleep(1000);


            LFspeed = (float) Range.clip(LFspeed, -maxspeed, maxspeed);
            LBspeed = (float) Range.clip(LBspeed, -maxspeed, maxspeed);
            RFspeed = (float) Range.clip(RFspeed, -maxspeed, maxspeed);
            RBspeed = (float) Range.clip(RBspeed, -maxspeed, maxspeed);
            robot.setDrivePower(LFspeed, LBspeed, RFspeed, RBspeed);


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
//    public class LinearSlideTest extends LinearOpMode {
//        private DcMotorEx motor1;
//        private DcMotorEx motor2;
//
//        boolean buttonPressed = false;
//        public static final int MAX_SLIDE_POSITION = 1000;
//
//        @Override
//        public void runOpMode() throws InterruptedException {
//            motor1 = hardwareMap.get(DcMotorEx.class, "SL");
//            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor1.setDirection(DcMotor.Direction.FORWARD);
//            motor1.setPower(0);
//            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            motor2 = hardwareMap.get(DcMotorEx.class, "SR");
//            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor2.setDirection(DcMotor.Direction.FORWARD);
//            motor2.setPower(0);
//            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//            waitForStart();

//            while (opModeIsActive()) {
//                if (!buttonPressed && gamepad1.a /*motor1.getCurrentPosition() < MAX_SLIDE_POSITION*/) {
//                    motor1.setPower(0.2);
//                    buttonPressed = true;
//                    motor2.setPower(0.2);
//                } else {
//                    motor1.setPower(0);
//                    motor2.setPower(0);
//                    buttonPressed = false;
//                }
//
//                telemetry.addData("Slide encoder", motor1.getCurrentPosition());

}
