function calibrateJamboxx(jamboxx)
% Calibrate the Jamboxx

    disp('Move mouthpiece to the left, then press enter...');
    pause;
    jamboxx.calibrateLeft();
    
    disp('Move mouthpiece to the right, then press enter...');
    pause;
    jamboxx.calibrateRight();
    
    disp('Move mouthpiece so that it points up, then press enter...');
    pause;
    jamboxx.calibrateTop();
    
    disp('Move mouthpiece so that it points down, then press enter...');
    pause;
    jamboxx.calibrateBottom();
    
    jamboxx.calibrateAir();

end

