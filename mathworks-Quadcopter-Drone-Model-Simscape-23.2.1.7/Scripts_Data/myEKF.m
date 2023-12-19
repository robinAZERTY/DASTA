% Implémentation du filtre de Kalman étendu avec calcul numérique des
% jacobiennes utilisant la méthode des différences finies.
% Écrit par robinAZERTY le 17/10/2023
% Commenté par chatGPT

classdef myEKF
    properties
        x_dim     % Dimension de l'état
        z_dim     % Dimension de la mesure
        u_dim     % Dimension de l'entrée de contrôle
        Un        % Vecteur d'entrée de contrôle
        Zn        % Vecteur de mesure
        R         % Matrice de covariance du bruit de mesure
        Xn        % Vecteur d'état estimé
        Pn        % Matrice de covariance de l'état estimé
        Xn_known  % Indicateur pour savoir si l'état est entièrement connu
        h         % Fonction de mesure
        H         % Jacobiennes de la fonction de mesure
        Q         % Matrice de covariance du bruit de processus
        I         % Matrice identité
        infinit   % Valeur d'infini
        f         % Fonction de transition d'état
        Fx        % Jacobiennes de la fonction de transition d'état par rapport à l'état
        Fu        % Jacobiennes de la fonction de transition d'état par rapport à la commande
        K         % Gain de Kalman
        S         % Matrice de covariance de la mesure
        y         % Résidu de mesure
        SI        % Inverse de la matrice de covariance de la mesure
        init      % Indicateur d'initialisation
    end
    
    methods
        function obj = myEKF(f, h, x_dim, z_dim, u_dim, Fx, Fu, H)
            % Initialisation de l'objet myEKF
            obj.x_dim = x_dim;
            obj.z_dim = z_dim;
            obj.u_dim = u_dim;
            obj.Un = zeros(u_dim, 1);
            obj.Zn = zeros(z_dim, 1);
            obj.R = eye(z_dim);
            obj.Xn = zeros(x_dim, 1);
            obj.Pn = zeros(x_dim, x_dim);
            obj.Xn_known = false;
            obj.h = h;
            obj.H = H;
            obj.Q = zeros(x_dim, x_dim);
            obj.I = eye(x_dim);
            obj.infinit = 1e7;
            obj.f = f;
            obj.Fx = Fx;
            obj.Fu = Fu;
            obj.K = zeros(x_dim, z_dim);
            obj.S = zeros(x_dim, x_dim);
            obj.y = zeros(z_dim, 1);
            obj.SI = zeros(z_dim, z_dim);
            obj.init = false;
            
            % Si les Jacobiennes ne sont pas spécifiées, les calculs numériques sont utilisés
            if isempty(Fx)
                obj.Fx = @(x, u) obj.Jacobian(@(x) obj.f(x, u), x, x_dim, x_dim);
            end
            if isempty(Fu)
                obj.Fu = @(u, x) obj.Jacobian(@(u) obj.f(x, u), u, x_dim, u_dim);
            end
            if isempty(H)
                obj.H = @(x) obj.Jacobian(obj.h, x, z_dim, x_dim);
            end
        end
        
        function j = Jacobian(~, vector_function, x, y_dim, x_dim)
            % Calcul numérique de la jacobienne d'une fonction vectorielle
            if nargin < 3
                y_dim = numel(vector_function(x));
            end
            if nargin < 4
                x_dim = numel(x);
            end
            j = zeros(y_dim, x_dim);
            delta = 1e-6;
            for i = 1:x_dim
                x1 = x;
                x1(i) = x1(i) + delta;
                x2 = x;
                x2(i) = x2(i) - delta;
                j(:, i) = ((vector_function(x1) - vector_function(x2)) / (2 * delta)).';
            end
        end
        
        function [obj] = predict(obj)
            % Prédiction de l'état et de la matrice de covariance
            Stx = obj.Fx(obj.Xn, obj.Un);
            Stu = obj.Fu(obj.Un, obj.Xn);
            obj.Xn = obj.f(obj.Xn, obj.Un);
            obj.Pn = Stx * obj.Pn * Stx.' + Stu * obj.Q * Stu.';
        end
       
        
        function [obj] = update(obj)
            % Mise à jour de l'estimation de l'état
            Hk = obj.H(obj.Xn);
            obj.y = obj.Zn - obj.h(obj.Xn);
            obj.S = Hk * obj.Pn * Hk.' + obj.R;
            obj.K = obj.Pn * Hk.' / obj.S;
            obj.Xn = obj.Xn + obj.K * obj.y;
            obj.Pn = (obj.I - obj.K * Hk) * obj.Pn;
        end
    end
end
