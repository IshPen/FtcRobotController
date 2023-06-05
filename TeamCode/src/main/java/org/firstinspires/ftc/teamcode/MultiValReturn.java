package org.firstinspires.ftc.teamcode;

public class MultiValReturn {
    XyhVector position;
    int currentLeftPos;
    int currentRightPos;
    int currentFrontPos;
    int currentBackPos;

    MultiValReturn(XyhVector pos, int currentLeftPosition, int currentRightPosition, int currentFrontPosition, int currentBackPosition){
        position = pos;
        currentLeftPos = currentLeftPosition;
        currentRightPos = currentRightPosition;
        currentFrontPos = currentFrontPosition;
        currentBackPos = currentBackPosition;
    }
}
