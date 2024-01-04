% Paramètres de la trajectoire
duration = 2; % Durée du signal en secondes
samplingRate = 1000; % Fréquence d'échantillonnage en Hz
A = 1;
B = 0.5;

% Paramètre temporel
t = linspace(0, duration, duration*samplingRate); % Crée un vecteur de 1000 points de 0 à 2*pi

% Équations paramétriques
x = A * sin(2*pi*t);
y = B * sin(2*pi*t) .* cos(2*pi*t);


amplitude = 0.05; % Amplitude de la marche aléatoire dérivable

derivableRandomWalkx = generateDerivableRandomWalk(duration, samplingRate, amplitude, 1);
derivableRandomWalky = generateDerivableRandomWalk(duration, samplingRate, amplitude, 1);

x = x+derivableRandomWalkx;
y = y+derivableRandomWalky;

% Affichage de la trajectoire
figure;
plot(x, y);
title('Trajectoire en 8');
xlabel('X');
ylabel('Y');
axis equal; % Pour s'assurer que les échelles sur les axes x et y sont égales
grid on;


function derivableRandomWalk = generateDerivableRandomWalk(duration, samplingRate, amplitude, cutoffFrequency)
    % Génère une marche aléatoire dérivable

    % Génère un bruit blanc
    whiteNoise = amplitude*sqrt(2/samplingRate) * randn(1, duration * samplingRate);
    

    % Intègre le bruit blanc pour obtenir une marche aléatoire
    randomWalk = cumsum(whiteNoise);

    order = 2; % Ordre du filtre

    [b, a] = butter(order, cutoffFrequency / (samplingRate / 2), 'low');

    % Appliquer le filtre à la marche aléatoire dérivable
    derivableRandomWalk = filter(b, a, randomWalk);

end
