Thanks buddy!
Basically, I have a project where I track light sources (components) on an image, as well as tracking point clouds with a LiDAR. I now wanted to unify the systems. I want the light source to "control" which point cloud is the person I'm tracking. 

Angle LiDAR: α_0  
Distance LiDAR: d_0
Angle track: α_1  

I'm planning on trying to calculate α_1 of a component for every distance d_0. Then I check for which distance d_0, the angle α_1 matches angle α_0 (or which is the closest match).
I get α_0 and d_0 for every single point cloud as correlated values. I only work with a 2D LiDAR!

My hypothetical process:

const FOCAL_LENGTH;
const MAX_ANGLE_DEVIATION;
const Vec2 IMAGE_CENTER;

var targetAngle
var α_0
var d_0
var α_1

for comp_l in LiDAR_components:
    α_0 <- comp_l.angle;
    d_0 <- comp_l.distance;

    for comp_TRK in TRK_components:
        α_1 <- atan((comp_TRK.x - IMAGE_CENTER.x) / FOCAL_LENGTH) * (180 / pi);
        
        if |α_1 - α_0| <= MAX_ANGLE_DEVIATION:
            targetAngle <- α_1;
            comp_l.validated <- True;

            break;

=> Target the comp_l, which was validated for tracking!