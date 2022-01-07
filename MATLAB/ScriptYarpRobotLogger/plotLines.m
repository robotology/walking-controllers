limitsY = ylim;
if isempty(initSS_left) && isempty(initSS_right)
    display('Always in double support!');
    line([timestamp(initDS(1)), timestamp(initDS(1))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);
    for indexPhase = 2:length(initDS)
        line([timestamp(initDS(indexPhase)), timestamp(initDS(indexPhase))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);
    end
elseif isempty(initSS_left)
    display('Always in right single support!');
    line([timestamp(initSS_right(1)), timestamp(initSS_right(1))], limitsY,'Color',[0.6, 0.4, 0.08],'LineStyle', '-.', 'linewidth', 3.5);
    for indexPhase = 2:length(initSS_right)
        line([timestamp(initSS_right(indexPhase)), timestamp(initSS_right(indexPhase))], limitsY,'Color',[0.6, 0.4, 0.08],'LineStyle', '-.', 'linewidth', 3.5);
    end
elseif isempty(nitSS_rigt)
    display('Always in left single support!');
    line([timestamp(initSS_left(1)), timestamp(initSS_left(1))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
    for indexPhase = 2:length(initSS_left)
        line([timestamp(initSS_left(indexPhase)), timestamp(initSS_left(indexPhase))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
    end
else
    line([timestamp(initDS(1)), timestamp(initDS(1))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);
    line([timestamp(initSS_right(1)), timestamp(initSS_right(1))], limitsY,'Color',[0.6, 0.4, 0.08],'LineStyle', '-.', 'linewidth', 3.5);
    line([timestamp(initSS_left(1)), timestamp(initSS_left(1))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
    
    for indexPhase = 2:length(initSS_left)
        line([timestamp(initSS_left(indexPhase)), timestamp(initSS_left(indexPhase))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
    end
    
    for indexPhase = 2:length(initSS_right)
        line([timestamp(initSS_right(indexPhase)), timestamp(initSS_right(indexPhase))], limitsY,'Color',[0.6, 0.4, 0.08],'LineStyle', '-.', 'linewidth', 3.5);
    end
    
    for indexPhase = 2:length(initDS)
        line([timestamp(initDS(indexPhase)), timestamp(initDS(indexPhase))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);
    end
end