function command = readVoiceCommand(voiceObj)
    
    if(voiceObj.BytesAvailable)
        fieldWidth = ['%',num2str(voiceObj.BytesAvailable),'c'];
        command = fscanf(voiceObj,fieldWidth);
    end
    
end