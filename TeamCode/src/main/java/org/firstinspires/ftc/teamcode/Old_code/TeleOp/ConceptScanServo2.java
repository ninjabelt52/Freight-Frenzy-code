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

package org.firstinspires.ftc.teamcode.Old_code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Scan Servo", group = "Concept")
@Disabled
public class ConceptScanServo2 extends LinearOpMode {

    static final double INCREMENT   = 0.05;
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members


    @Override
    public void runOpMode() {

        Servo   servo;
        servo = hardwareMap.get(Servo.class, "horizontal");
        Servo   servo2;
        servo2 = hardwareMap.get(Servo.class, "vertical");
        double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        double  position2 = (MAX_POS - MIN_POS) / 2;



        while(opModeIsActive()){

            if(gamepad1.dpad_left){
                 position = position-INCREMENT;
            }
            if(position<MIN_POS){
                position = MIN_POS;
            }
            if(gamepad1.dpad_right){
                position = position+INCREMENT;
            }
            if(position>MAX_POS){
                position = MAX_POS;
            }

            if(gamepad1.dpad_down){
                position2 = position2-INCREMENT;
            }
            if(position2<MIN_POS){
                position2 = MIN_POS;
            }
            if(gamepad1.dpad_up){
                position2 = position2+INCREMENT;
            }
            if(position2>MAX_POS){
                position2 = MAX_POS;
            }

            servo.setPosition(position);
            servo2.setPosition(position2);
            telemetry.addData("Horizontal Position", "%5.2f", position);
            telemetry.addData("Vertical Position", "%5.2f", position2);
            telemetry.update();
        }




    }
}
