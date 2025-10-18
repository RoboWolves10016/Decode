package org.firstinspires.ftc.teamcode;

public class Robot {

    private static Robot instance;
    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot() {}

}
