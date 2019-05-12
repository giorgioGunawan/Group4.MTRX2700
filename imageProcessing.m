% Group 4 - 642 
% Giorgio Gunawan
%
% This code will take values in meters from the LIDAR 
% It will actegorize in the matter of:
% - Distance = x where : 
%   + {0 - 1.3} = 1;
%   + {1.3 - 40} = 0;

% CLEANING UP ==============================================
clf;
clf reset;
clear;
clc;

% SETUP ====================================================
% Each lidar reading represents a movement. From calculation:
% Resolution = 0.5 deg
% Azimuth = -50 to 50
% Elevation = -25 to 25
% Hence the Azimuth (X Axis) = (50 -(-50))/0.5 = 200 steps
% And the Elevation (Y Axis) = (25 -(-25))/0.5 = 100 steps
%
% Plan:
% Two for loops, x and then y. fill it going down so..
xMax = 8; %= 200 
yMax = 4; %= 100

% START ====================================================
% First we get value s first! 

% Set terminator to CR so it waits longer and 
% doesnt produce errors
s = serial('COM6','BaudRate',9600);

% Set terminator to CR so it waits longer and 
% doesnt produce errors
set(s,'Terminator','CR');

% Set the image matrix to an empty matrix and then print it out
imageMatrix = zeros(yMax,xMax);
printImg(imageMatrix);

for i = 1:xMax
    for j = 1:yMax
        % OPEN PORT ===============================================
        % Then we open the serial port 
        fopen(s);

        % Now get the information being sent!
        % Then we get the information being sent
        a = fgets(s);

        % Close the file
        fclose(s);
        
        % Change number collected from string to number
        number = str2num(a);
        if (number > 1.4)
            imageMatrix(j,i) = 0;
        elseif (number <= 1.4)
            imageMatrix(j,i) = 1;
        else
            disp("Undefined - Out of Range");
        end
        
        % Printing in real time
        printImg(imageMatrix);
    end
end
% CLOSE PORT ==============================================

% FINAL --> Show as an image
printImg(imageMatrix);

% Function to print image
function printImg(matrix)
    figure(1);
    imagesc(matrix)
    caxis([0,1]);
    colormap(flipud(gray));
    %colorbar;
end
