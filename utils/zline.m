function h = zline(z_value, varargin)
    % zline - Adds a horizontal line in the z-axis at the specified z_value
    % 
    % Syntax: 
    %   h = zline(z_value)
    %   h = zline(z_value, lineStyle)
    %   h = zline(z_value, lineStyle, label)
    % 
    % Inputs:
    %   z_value - The z-axis value where the line should be drawn.
    %   varargin - (Optional) Additional arguments: lineStyle and label.
    %              lineStyle (string) - Line style (e.g., '--r' for dashed red).
    %              label (string) - Label text to display near the line.
    %
    % Outputs:
    %   h - Handle to the created line object, for further modification.
    %
    % Example:
    %   % Create a 3D surface plot and add a zline at z = 0
    %   [X,Y] = meshgrid(1:10, 1:10);
    %   Z = peaks(10);
    %   figure;
    %   surf(X, Y, Z);    % 3D surface plot
    %   h = zline(0, '--r', 'z = 0');  % Dashed red line at z = 0 with label
    %
    %   % Modify the line (optional, using the handle)
    %   set(h, 'LineWidth', 2);

    % Parse input arguments
    lineStyle = '-';  % Default solid line
    label = '';

    % Check if varargin contains lineStyle and/or label
    if nargin >= 2
        lineStyle = varargin{1};  % Line style from input
    end
    if nargin >= 3
        label = varargin{2};  % Label from input
    end
    
    % Hold on to add the zline on existing plot
    hold on;
    
    % Get current axes limits
    x_limits = get(gca, 'XLim');
    y_limits = get(gca, 'YLim');
    
    % Create a 3D line on the z-axis with the given z_value
    h = plot3(x_limits, , [z_value, z_value], lineStyle, 'LineWidth', 1.5);
    
    % Optionally add a label near the line
    if ~isempty(label)
        text(mean(x_limits), y_limits(1), z_value, label, 'HorizontalAlignment', 'center');
    end
    
    % Return handle for further customization
    hold off;
end