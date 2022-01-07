% close all

phases;

energy = struct();

joints_plotted = 0;

for part = fieldnames(parts)'
    figure;
    
    isNewFig = 1;
    
    hold on;
    
    l = length(parts.(part{:}));
    
    cols = 0;
    rows = 0;
    
    if(~mod(l,3))
        rows = 3;
        cols = l / 3;
    end
    
    if(~mod(l,2))
        rows = 2;
        cols = l / 2;
    end
    
    if(~mod(l,7))
        rows = 3;
        cols = 3;
    end
    
    i = 1;
    
    energy.(part{:}) = [];
    
    for joint = parts.(part{:})
        
        label = {};
        
        subplot(rows, cols, i)
        
        jointValue = joint{:};
        plot(timestamp, joint_pos.desired(joints_plotted+i,:)*180/pi, timestamp, joint_pos.retargeting(joints_plotted+i,:)*180/pi, timestamp, joint_pos.measured(joints_plotted+i,:)*180/pi)
        hold on;
        label = [label, 'desired', 'retargeting', 'measured'];
        
        str = regexprep(jointValue, '_', ' ');
        
        if(isNewFig)
            plot_aesthetic(str, 'time (s)' , 'Joint pos ($deg$)', '', label{:},  'Init SS Right', 'Init SS Left', 'Init DS');
            %       xlim([204,210]);
            
            isNewFig = 0;
        else
            plot_aesthetic(str, 'time (s)' , 'Joint pos ($deg$)', '');
            %       xlim([204,210]);
            
        end
        i = i + 1;
    end
    
    joints_plotted = l + joints_plotted;
end