function [linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = getUserInput(jamboxx,jamsterMode,w)

    x = jamboxx.X;
    y = jamboxx.Y;
    air = jamboxx.Air;
    
    linVel_L = [0;0;0];
    linVel_R = [0;0;0];
    angVel_L = [0;0;0];
    angVel_R = [0;0;0];
    wristVel_L = [];
    wristVel_R = [];
    grip_L = [];
    grip_R = [];
    
    switch jamsterMode
        case 'wheelchair'
            % add w.Drive
        case 'baxterTrans'
            % change linVel
        case 'baxterRot'
            % change angVel
        otherwise
    end
    
end
