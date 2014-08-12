function newVal = limitVal(lowLim,upLim,val)

    if val>upLim
        newVal = upLim;
    elseif val<lowLim
        newVal = lowLim;
    else
        newVal = val;
    end

end
