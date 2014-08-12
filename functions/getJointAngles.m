function [jointAnglesLeft,jointAnglesRight] = getJointAngles(baxter_JointStates)
% Get current joint angles from both arms and sorts them in the right order

    % Gather joint information
    i = 1;
    while(i)  
        if baxter_JointStates.InValueValid == 1 
            jointStates = baxter_JointStates.InValue;
            jointAnglesLeft = jointStates.position(3:9);
            jointAnglesRight = jointStates.position(10:16);
            i=0;    
        end  
    end
    
    % Reorder jointAngle to be [left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2]
    joint34 = jointAnglesLeft(3:4);
    joint12 = jointAnglesLeft(1:2);
    jointAnglesLeft = [joint34;joint12;jointAnglesLeft(5:7)];
    
    % Reorder jointAngle to be [right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2]
    joint34 = jointAnglesRight(3:4);
    joint12 = jointAnglesRight(1:2);
    jointAnglesRight = [joint34;joint12;jointAnglesRight(5:7)];
        
end