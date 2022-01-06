function plot_aesthetic(Title, Label_x, Label_y, Label_z, varargin)
% PLOT_AESTHETIC add Title, label and legends in a plot
%   PLOT_AESTHETIC(Title, Label_x, Label_y, Label_z, Legend_1, ..., Legend_n)
%   add title, labels and legends in a plot. LaTex syntax is allowed.

% set labels
if ~isempty(Label_x)
    x_label = xlabel(Label_x);
    set(x_label, 'Interpreter', 'latex');
    set(x_label, 'FontSize', 16);
end

if ~isempty(Label_y)
    y_label = ylabel(Label_y);
    set(y_label,'Interpreter','latex');
    set(y_label,'FontSize', 16);
end

if ~isempty(Label_z)
    z_label = zlabel(Label_z);
    set(z_label,'Interpreter','latex');
    set(z_label,'FontSize', 16);
end

% set legend
if ~isempty(varargin)
    % get the legend object
    leg = get(legend(gca),'String');
    
    % if the legend does not exist create a new one
    
    for i = 1:length(varargin)
        varargin(i) = strrep(varargin(i),'_',' ');
    end
    
    if isempty(leg)
        new_legend = varargin;
    else
        old_legend = leg;
        % when a new plot is draw an automatic string is added to the
        % legend
        new_legend = [old_legend(1:end-1), varargin{:}];
    end
%     h = legend(varargin, 'Location', 'northoutside', 'Orientation','horizontal');
%     h = legend(varargin, 'Location', 'best');
    h = legend(varargin, 'Location', [0.93, 0.5, .05 .055], 'Orientation','vertical');
    set(h,'Interpreter','latex')
    set(h,'FontSize', 13);
end

% change linewidth
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5)

% set the title
if ~isempty(Title)
    tit = title(Title);
    set(tit,'FontSize', 20);
    set(tit,'Interpreter','latex');
end

% change font size
set(gca,'FontSize', 16)

% set grid
grid on;
end