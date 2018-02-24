I = imread('data.pgm');
imshow(I)
figure
imshow(I)

Rmin = 5;
Rmax = 30;

% Find all the bright circles in the image
[centers, radii, metrics] = imfindcircles(I,[Rmin Rmax], ...
    'ObjectPolarity','bright', ...
    'Method', 'TwoStage', ...    // PhaseCode or TwoStage
    'Sensitivity', 0.9 ...      // default 0.85    
);     

% Plot bright circles in blue
% viscircles(centers, radii,'Color','g');
viscircles(centers, radii);
