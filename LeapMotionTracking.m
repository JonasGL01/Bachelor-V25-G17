
BoomHos = [data.mcp_Z] - [data.meta_Z];
BoomMot = [data.mcp_Y] - [data.meta_Y];
BoomHyp = sqrt(BoomHos.^2 + BoomMot.^2);
BoomAngleOld = asin(BoomMot ./ BoomHyp)';
BoomAngle = 0.93 .* BoomAngleOld + 0.92;

ArmHos = [data.pip_Z] - [data.mcp_Z];
ArmMot = [data.pip_Y] - [data.mcp_Y];
ArmHyp = sqrt(ArmHos.^2 + ArmMot.^2);
ArmAngleOld = asin(ArmMot ./ ArmHyp)';
ArmAngle = 0.69 .* ArmAngleOld + 2.43;

BucHos = [data.tip_Z] - [data.pip_Z];
BucMot = [data.tip_Y] - [data.pip_Y];
BucHyp = sqrt(BucHos.^2 + BucMot.^2);
BucAngleOld = acos(BucHos ./ BucHyp)';
BucAngle = 0.7 .* BucAngleOld + 1.12;

RotHos = [data.mcp_Z] - [data.meta_Z];
RotMot = [data.mcp_X] - [data.meta_X];
RotHyp = sqrt(RotHos.^2 + RotMot.^2);
RotAngleOld = asin(RotMot ./ RotHyp)';
RotAngle = 1.13 * RotAngleOld + 0.48;

Time = [data.time]';

plot(BucAngle)
hold on
plot(ArmAngle)
hold on
plot(BoomAngle)
hold on
plot(RotAngle)
legend("Bucket", "Arm", "Boom", "Rot");

Angles = [round(Time,1), round(BoomAngle,1), round(ArmAngle,1), round(BucAngle,1), round(RotAngle,1)];

writematrix(Angles, 'Angles.xls')

BoomMin = min(BoomAngle);
BoomMax = max(BoomAngle);
ArmMin = min(ArmAngle);
ArmMax = max(ArmAngle);
BucMin = min(BucAngle);
BucMax = max(BucAngle);
RotMax = max(RotAngle);
RotMin = min(RotMax);

BoomDelta = BoomMax - BoomMin;
ArmDelta = ArmMax - ArmMin;
BucDelta = BucMax - BucMin;
RotDelta = RotMax - RotMin;

