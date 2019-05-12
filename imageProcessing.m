% Group 4 - 642 
% Giorgio G
%
% LIDAR SHAPE DETECTION - REAL TIME DRAWING
%
% This code will take values in meters from the LIDAR 
% It will actegorize in the matter of:
% - Distance = x where : 
%   + {0 - 1.3} = 1;
%   + {1.3 - 40} = 0;
% Color Code:
% BLACK --> Shape
% WHITE --> Background
% GREY  --> ERROR

% SETTINGS =================================================
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
s = serial('COM10','BaudRate',9600);

% Set terminator to CR so it waits longer and 
% doesnt produce errors
set(s,'Terminator','CR');

% Set the image matrix to an empty matrix and then print it out
imageMatrix = zeros(yMax,xMax);
printImg(imageMatrix);

% PRINT THE SHAPE =================================================
% For the image, the tracing will be done in a zigzag pattern
% hence there should be a counter whereby it alternates from 
% going from top to bottom (1:yMax) and bottom to top (yMax:-1:1)
%
% If odd: Goes from top to bottom
% If even: Goes from bottom to top
yDirection = 1;

for i = 1:xMax
    
    % If odd, do top to bottom 
    if mod(yDirection,2)==1
        for j = 1:yMax

            % Printing in real time
            printImg(imageMatrix);

            % OPEN PORT ===============================================
            % Then we open the serial port 
            fopen(s);

            % Now get the information being sent!
            % Then we get the information being sent
            a = fgets(s);

            % Close the file
            fclose(s);
            % CLOSE PORT ==============================================

            % Change number collected from string to number
            number = str2num(a);

            % If distance is greater than 1.4 meters, there is
            % no object there and the matrix stays 0
            if (number > 1.4 & number <= 40)
                imageMatrix(j,i) = 0;

            % If distance is less than 1.4, there is an object
            elseif (number <= 1.4)
                imageMatrix(j,i) = 1;

            % In circumstances where the nearest thing is more
            % than 40m away, give it a color of 0.5 where it will
            % have the color grey and hence is an easily spotted
            % error
            else
                imageMatrix(j,i) = 0.5;
            end

            % Printing in real time
            % printImg(imageMatrix);
        end
        
    % If y direction is even, the pattern goes from bottom to top
    else
        for j = yMax:-1:1

            % Printing in real time
            printImg(imageMatrix);

            % OPEN PORT ===============================================
            % Then we open the serial port 
            fopen(s);

            % Now get the information being sent!
            % Then we get the information being sent
            a = fgets(s);

            % Close the file
            fclose(s);
            % CLOSE PORT ==============================================

            % Change number collected from string to number
            number = str2num(a);

            % If distance is greater than 1.4 meters, there is
            % no object there and the matrix stays 0
            if (number > 1.4 & number <= 40)
                imageMatrix(j,i) = 0;

            % If distance is less than 1.4, there is an object
            elseif (number <= 1.4)
                imageMatrix(j,i) = 1;

            % In circumstances where the nearest thing is more
            % than 40m away, give it a color of 0.5 where it will
            % have the color grey and hence is an easily spotted
            % error
            else
                imageMatrix(j,i) = 0.5;
            end

            % Printing in real time
            % printImg(imageMatrix);
        end
    end
    
    % Increment yDirection to toggle odd -> even or vice versa
    yDirection = yDirection + 1;
end

% Show final image 
printImg(imageMatrix);
% END PRINT THE SHAPE =============================================

% Function to print image
function printImg(matrix)

    % Plot in figure 1
    figure(1);
    
    % Print the matrix
    imagesc(matrix)
    
    % Limit the scale:
    % Minimum = 0, Maximum = 1
    caxis([0,1]);
    
    % Use grayscale color and flip it to make black (1)/on
    % and white (0)/off
    colormap(flipud(gray));
end
