%% Write Animated GIF
% Draw a series of plots, capture them as images, and write them into one animated 
% GIF file.

% Save the nine images into a GIF file. Because three-dimensional data is not 
% supported for GIF files, call |rgb2ind| to convert the RGB data in the image 
% to an indexed image |A| with a colormap |map|. To append multiple images to 
% the first image, call |imwrite| with the name-value argument |WriteMode| set 
% to |"append"|.
nImages = 17;

filename = fullfile("figure", "crashing.gif"); % Specify the output file name
for idx = 1:nImages
    img_path = fullfile("figure", "crashing", sprintf("%03d.png", idx)) ;
    cur_img = imread(img_path);
    [A, map] = rgb2ind(cur_img, 256);
    if idx == 1
        imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",1);
    else
        imwrite(A,map,filename,"gif","WriteMode","append", "DelayTime", 0.1);
    end
end
%% 
% |imwrite| writes the GIF file to your current folder. Setting |LoopCount| 
% to |Inf| causes the animation to loop continuously. Setting |DelayTime| to |1| 
% specifies a 1-second delay between the display of each image in the animation.
% 
% 
% 
% _Copyright 2012-2022 The MathWorks, Inc._