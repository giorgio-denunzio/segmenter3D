function findcolumnloop(pgmname)
% Compile with
% mcc -m   -d files findcolumnloop
% (after creating a directory called "files")

% https://it.mathworks.com/matlabcentral/answers/92537-how-do-i-pass-arguments-into-and-out-of-my-standalone-executable


warning off;
% I = imread('data.pgm');
I = imread(pgmname);
figure1 = figure('color', 'w'); imshow(I);

Rmin = 3;   % 5 for GH, 3 for Apollo
Rmax = 30;

sens = 0.700:0.005:1;  % se metto 0.002 non funziona ovviamente piu' perche' ho un plateau anche a 6 colonne!!!
foundCol = [];
for s = sens
    % Find all the bright circles in the image
    [centers, radii, metrics] = imfindcircles(I,[Rmin Rmax], ...
        'ObjectPolarity','bright', ...
        'Method', 'TwoStage', ...    // PhaseCode or TwoStage
        'Sensitivity', s ...      // default 0.85
        );
    foundCol = [foundCol length(centers)];
end
figure, plot(sens, foundCol, '+')

% devo trovare i periodi di valore costante! Le istruzioni seguenti
% dapprima scrivono 0 nella parte di dati in cui il numero di circonferenze
% trovate e' mantenuto costante, e 1 dove c'e' stata la variazione:
%  foundCol  = [0 0 0 0 0 1 1 1 1 1 1 1 3 3 3 3 3 4 4 4 4 4 4 4 5 6 6 7]
% da':
%  dFoundCol = [0 0 0 0 1 0 0 0 0 0 0 2 0 0 0 0 1 0 0 0 0 0 0 1 1 0 1]
% Poi 
%  idx       = [5    12    17    24    25    27]
%  idxGt5    = [1     1     1     1     0     0]
%  p = 24 e ora foundCol(24) e' l'ultimo elemento del tratto grosso modo
%  orizzontale della curva, prendo la corrispondente sensibilita':
%  sensOK = sens(p)

% https://it.mathworks.com/matlabcentral/answers/8299-length-of-the-longest-continuous-string-of-the-same-number
% suggerimenti non usati

dFoundCol = diff(foundCol);
dFoundColIsNot0 = find(dFoundCol);
idx = diff([0 dFoundColIsNot0]);
idxGe5 = idx >= 5;    % la soglia a 5 e' del tutto arbitraria perche' i valori effettivamente osservati sono >= 15.
p = sum(idx(idxGe5));
sensOK = sens(p)

figure(figure1)

% Find all the bright circles in the image
[centers, radii, metrics] = imfindcircles(I,[Rmin Rmax], ...
    'ObjectPolarity','bright', ...
    'Method', 'TwoStage', ...    // PhaseCode or TwoStage
    'Sensitivity', sensOK ...      // default 0.85    
);     

% Plot bright circles in blue
% viscircles(centers, radii,'Color','g');
viscircles(centers, radii);

save centers.txt centers -ascii
save radii.txt radii -ascii
pause(2)
close all

warning on



% Plot bright circles in blue
% viscircles(centers, radii,'Color','g');
%viscircles(centers, radii);









