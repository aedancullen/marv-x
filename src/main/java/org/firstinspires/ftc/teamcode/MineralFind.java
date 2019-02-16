/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class MineralFind {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "Ac0A5xL/////AAABmbaZRuKrykmMhgpBAfm4wxkWMeMkHp/ij0Bv8cnqyigZaQN4qUU9wK+CmT4WDTRnZef/AEyluCOS1Z8a5pwiHeJpjLNqVcQoQsXBJT06NyKXZ2v2BDMqURXAnLCl82w+vIY3u4W/XdtFBt2m0/5OQNLFZRaIz3LJZaXGYz4hSRFAyMj0yVonukAXvjQljMxjd1YNUhpXk8V3qJaXS49Ep69t0AypLu+hE2AdHg1e15q29AifPAANhWM0PpWEACCVn7RWe19wyNi6N8Ab0c77kudZoGWmQF4hZVGRKK3ZrVz7kz1wyk3tfzHUsteJm7hbw8kagADt2ZKBDkO4+0i0HtB2hXcrKUp/w23nNTtY4SJ0";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    HardwareMap hardwareMap;

    public MineralFind(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    private List<Recognition> getMineralsIn(float t, float b, float l, float r, List<Recognition> recs) {
        List<Recognition> mineralsIn = new ArrayList<Recognition>();
        for (Recognition rec : recs) {
            if (rec.getTop() < b && rec.getBottom() > t && rec.getLeft() < r && rec.getRight() > l) {
                mineralsIn.add(rec);
            }
        }
        return mineralsIn;
    }

    private List<Recognition> getAllGolds(List<Recognition> recs) {
        List<Recognition> golds = new ArrayList<Recognition>();
        for (Recognition rec : recs) {
            if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) {
                golds.add(rec);
            }
        }
        return golds;
    }

    private List<Recognition> filterMineralLine(List<Recognition> recs) {
        for (Recognition rec : getAllGolds(recs)) {
            List<Recognition> aligned = getMineralsIn(rec.getTop(), rec.getBottom(), 0, rec.getImageWidth(), recs);
            if (aligned.size() == 3) {
                return aligned;
            }
        }
        return new ArrayList<Recognition>();
    }

    public void detectInit() {
        initVuforia();
        initTfod();
        tfod.activate();
    }

    public int detectLoop() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            updatedRecognitions = filterMineralLine(updatedRecognitions);
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        return 0;
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        return 2;
                    } else {
                        return 1;
                    }
                }
            }
        }
        return -1;
    }

    public void detectStop() {
        tfod.deactivate();
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
