function setBaxterConstants
% Set baxter constants like position vectors, joint types, etc.

    x = [1;0;0]; y = [0;1;0]; z = [0;0;1];

    % NOTE:
    % - Origin is at the base of first joint for both arms
    % - Zero configuration is when the joint_states.position reads 0 for all
    %   joints.
    % - Y-axis is in the direction the arm is stretched
    %   out in zero configuration, Z-axis points straight up

    
    % Setting up left arm parameters
    leftArm.P = [[0;0;0],[0;0.1;0.235],[0;0;0],[0;0.24;-0.095],[0;0.09;0],[0;0.28;-0.02],[0;0.18;0],[0;0.17;0]];
    leftArm.H = [z,-x,y,-x,y,-x,y];
    leftArm.type = [0,0,0,0,0,0,0];
    leftArm.n = 7;
    baxterconst.leftArm = leftArm;
    
    % Setting up right arm parameters
    rightArm.P = [[0;0;0],[0;0.1;0.235],[0;0.1;0],[0;0.24;-0.095],[0;0.09;0],[0;0.28;-0.02],[0;0.18;0],[0;0.17;0]];
    rightArm.H = [z,-x,y,-x,y,-x,y];
    rightArm.type = [0,0,0,0,0,0,0];
    rightArm.n = 7;
    baxterconst.rightArm = rightArm;

    % Setting up joint angle limits
    baxterconst.jointLimitsLow = [-1.702, -1.419, -3.054, -0.042, -3.059, -1.571, -3.059, ...
                                -1.702, -1.419, -3.054, -0.042, -3.059, -1.571, -3.059]';
    baxterconst.jointLimitsHigh = [1.455, 1.047, 3.054, 2.602, 3.059, 2.094, 3.059, ...
                                1.455, 1.047, 3.054, 2.602, 3.059, 2.094, 3.059]';
    
    % Setting up joint velocity limits
    baxterconst.jointVelLimit=[2.5 2.5 2.5 2.5 5 5 5 ...
                                2.5 2.5 2.5 2.5 5 5 5]';

assignin('base','baxterConst',baxterconst);