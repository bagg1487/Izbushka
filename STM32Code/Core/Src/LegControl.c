#include "LegControl.h"
#include <stdlib.h>


ArmAngles setArmAngle(float A, float B, float C) {
    ArmAngles angles = {A, B, C};
    return angles;
}

int getMaxAngleArm(ArmAngles current, ArmAngles target) {
    int deltaA = abs(current.A - target.A);
    int deltaB = abs(current.B - target.B);
    int deltaC = abs(current.C - target.C);
    
    int max = deltaA;
    if (deltaB > max) max = deltaB;
    if (deltaC > max) max = deltaC;
    return max;
}

ArmAngles getNextStepArm(ArmAngles current, ArmAngles target, int maxAngle, float step) {
    ArmAngles result;
    
    if (maxAngle > 0) {
        result.A = current.A + step * (target.A - current.A) / maxAngle;
        result.B = current.B + step * (target.B - current.B) / maxAngle;
        result.C = current.C + step * (target.C - current.C) / maxAngle;
    } else {
        result = target;
    }
    
    return result;
}

bool targetCheckArm(ArmAngles current, ArmAngles target) {
    return (current.A == target.A && 
            current.B == target.B && 
            current.C == target.C);
}


LegAngles setLegAngle(float B, float C) {
    LegAngles angles = {B, C};
    return angles;
}

int getMaxAngleLeg(LegAngles current, LegAngles target) {
    int deltaB = abs(current.B - target.B);
    int deltaC = abs(current.C - target.C);
    
    return (deltaB > deltaC) ? deltaB : deltaC;
}

LegAngles getNextStepLeg(LegAngles current, LegAngles target, int maxAngle, float step) {
    LegAngles result;
    
    if (maxAngle > 0) {
        result.B = current.B + step * (target.B - current.B) / maxAngle;
        result.C = current.C + step * (target.C - current.C) / maxAngle;
    } else {
        result = target;
    }
    
    return result;
}

bool targetCheckLeg(LegAngles current, LegAngles target) {
    return (current.B == target.B && current.C == target.C);
}