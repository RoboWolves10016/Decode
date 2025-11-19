package org.firstinspires.ftc.teamcode.util;

public enum Pattern {
    GPP(21,0),
    PGP(22,1),
    PPG(23, 2);


    public int obeliskID;
    public int greenIndex;
    Pattern(int obeliskID, int greenIndex) {
        this.obeliskID = obeliskID;
        this.greenIndex = greenIndex;
    }
}
