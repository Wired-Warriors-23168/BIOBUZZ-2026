package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerMechanism {

        private Servo flowerLeft;
        private Servo flowerRight;
        double flowerPosMin = 0;
        double flowerPosMax = 1;
        double flowerDownPos = 0.4;

        public FlowerMechanism(HardwareMap hwMap) {
            flowerLeft = hwMap.get(Servo.class, "leftFlower");
            flowerLeft.setDirection(Servo.Direction.REVERSE);
            flowerRight = hwMap.get(Servo.class, "rightFlower");
            flowerRight.setDirection(Servo.Direction.REVERSE);
        }

        public FlowerMechanism(HardwareMap hwMap, boolean isRamen) {
            flowerRight = hwMap.get(Servo.class, "rightFlower");
            flowerRight.setDirection(Servo.Direction.REVERSE);
            flowerLeft = hwMap.get(Servo.class, "leftFlower");
            flowerLeft.setDirection(Servo.Direction.FORWARD);

            if (isRamen) {

            } else {

            }
        }
        public void setFlowerPosUp() {
            flowerRight.setPosition(.075);
            flowerLeft.setPosition(.075);
        }
        public void setFlowerPosDown() {
            flowerRight.setPosition(flowerDownPos);
            flowerLeft.setPosition(flowerDownPos);
        }
        public double getFlowerPos() {
            return flowerLeft.getPosition();
        }
    }
