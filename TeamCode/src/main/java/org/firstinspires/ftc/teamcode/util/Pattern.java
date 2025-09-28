package org.firstinspires.ftc.teamcode.util;

public enum Pattern {
    GPP(21,1),
    PGP(22,2),
    PPG(23, 3);


    public int obeliskID;
    public int greenIndex;
    Pattern(int obeliskID, int greenIndex) {
        this.obeliskID = obeliskID;
        this.greenIndex = greenIndex;
    }
}
