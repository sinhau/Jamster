function [linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R] = getUserInput(jamboxx,jamsterMode,w)

    x = jamboxx.X;
    y = jamboxx.Y;
    air = jamboxx.Air;
    
    if strcmp(jamsterMode,'baxterTransLeft') || strcmp(jamsterMode,'baxterTransRight') || strcmp(jamsterMode,'baxterRotLeft') || strcmp(jamsterMode,'baxterRotRight')
        if x > 0.4
            x = limitVal(0.1,0.7,x)*0.1;
        elseif x < -0.4
            x = limitVal(-0.7,-0.1,x)*0.1;
        else
            x = 0;
        end

	if y > 0.15
	   y = limitVal(0.5,0.5,y)*-0.1;
	elseif y < -0.15
	   y = limitVal(-0.5,-0.5,y)*-0.1;
	else
	   y = 0;
	end

        if air > 0.1
            air = limitVal(0.1,0.7,air)*0.1;
        elseif air < -0.1
            air = limitVal(-0.7,-0.1,air)*0.1;
        else
            air = 0;
        end
    end

    linVel_L = [0;0;0];
    linVel_R = [0;0;0];
    angVel_L = [0;0;0];
    angVel_R = [0;0;0];
    wristVel_L = [];
    wristVel_R = [];

    switch jamsterMode
        case 'wheelchair'
            if air > 0.1
                if (x > -0.4) && (x < 0.4)
                    xInput = 127;
                    yInput = 142.222*air + 112.778;
                    yInput = limitVal(0,127,255-yInput);
                elseif x > 0.4
                    xInput = -141.111*air + 141.111;
                    xInput = limitVal(0,127,xInput);
                    yInput = 127;
                elseif x < -0.4
                    xInput = 142.222*air + 112.778;
                    xInput = limitVal(127,255,xInput);
                    yInput = 127;
                else
                    xInput = 127;
                    yInput = 127;
                end
            elseif air < -0.1
                if (x > -0.4) && (x < 0.4)
                    xInput = 127;
                    yInput = 141.111*air + 141.111;
                    yInput = limitVal(127,255,255-yInput);
		elseif x > 0.4
		    xInput = 142.222*air + 112.778;
  		    xInput = limitVal(127,255,xInput);
                    yInput = 127;
		elseif x < -0.4
		    xInput = -141.111*air + 141.111;
		    xInput = limitVal(0,127,xInput);
		    yInput = 127;
                else
                    xInput = 127;
                    yInput = 127;
                end
            else
		xInput = 127;
		yInput = 127;
	    end
            w.Drive(uint8(xInput),int8(1));
            w.Drive(uint8(yInput),int8(-1));
        case 'baxterTransLeft'
	    if (y ~= 0 && air ~= 0) 
		linVel_L = [0;0;air];
	    elseif (x ~= 0 && air ~= 0)
		linVel_L = [0;sign(air)*x;0];
	    elseif (x == 0 && air ~= 0)
		linVel_L = [air;0;0];
	    else
		linVel_L = [0;0;0];
	    end
        case 'baxterTransRight'
	    if (y ~= 0 && air ~= 0)
		linVel_R = [0;0;air];
	    elseif (x ~= 0 && air ~= 0)
		linVel_R = [0;sign(air)*x;0];
	    elseif (x == 0 && air ~= 0)
		linVel_R = [air;0;0];
	    else
		linVel_R = [0;0;0];
	    end
        case 'baxterRotLeft'
            angVel_L = [air;y;x];
        case 'baxterRotRight'
            angVel_R = [air;y;x];
        case 'baxterWrist'
            if air > 0.1
                if (x > 0.7)
                    wristVel_L = [0;0;1];
                elseif (x > 0.36) && (x < 0.65)
                    wristVel_L = [0;0.7;0];
                elseif (x > 0.03) && (x < 0.3)
                    wristVel_L = [0.7;0;0];
                elseif (x > -0.3) && (x < -0.03)
                    wristVel_R = [0.7;0;0];
                elseif (x > -0.65) && (x < -0.36)
                    wristVel_R = [0;0.7;0];
                elseif (x < -0.7)
                    wristVel_R = [0;0;1];
                end
            elseif air < -0.1
                if (x > 0.7)
                    wristVel_L = [0;0;-1];
                elseif (x > 0.36) && (x < 0.65)
                    wristVel_L = [0;-0.7;0];
                elseif (x > 0.03) && (x < 0.3)
                    wristVel_L = [-0.7;0;0];
                elseif (x > -0.3) && (x < -0.03)
                    wristVel_R = [-0.7;0;0];
                elseif (x > -0.65) && (x < -0.36)
                    wristVel_R = [0;-0.7;0];
                elseif (x < -0.7)
                    wristVel_R = [0;0;-1];
                end
            end
    end

end
