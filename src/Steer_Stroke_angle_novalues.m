% =========================================================================
% Simulación: Steering Angle vs Stroke (FSAE)
% =========================================================================
clear; clc; close all;

% 1. Parámetros Geométricos (Valores de ejemplo - ¡Necesitas medirlos en CAD!)
W_kp = 1200;       % Distancia transversal entre Kingpins (mm)
L_rack = 400;      % Longitud de la cremallera (mm)
Y_rack = 150;      % Distancia longitudinal del eje a la cremallera (mm)
R_arm = 100;       % Longitud del steering arm en la mangueta (mm)
L_tie = 420;       % Longitud de la bieleta / tie rod (mm)
theta_0 = deg2rad(105); % Ángulo inicial estático del steering arm

% 2. Vector de entrada: Desplazamiento del rack (Stroke)
% Supongamos que el rack se mueve de -25 mm a +25 mm
stroke = linspace(-25, 25, 100); 
steering_angle_deg = zeros(1, length(stroke));

% 3. Solución Cinemática (Loop)
% Opciones para el solver numérico
options = optimset('Display', 'off'); 

for i = 1:length(stroke)
    s = stroke(i);
    
    % Posición del extremo del rack R(s)
    Rx = (L_rack / 2) + s;
    Ry = Y_rack;
    
    % Función de restricción de distancia (Tie Rod = constante)
    % Buscamos el ángulo 'theta' que hace que esta ecuación sea 0
    % Ecuación: (Bx - Rx)^2 + (By - Ry)^2 - L_tie^2 = 0
    fun = @(theta) ((W_kp/2 + R_arm*cos(theta)) - Rx)^2 + ...
                   ((0      + R_arm*sin(theta)) - Ry)^2 - L_tie^2;
               
    % Resolver numéricamente usando el ángulo estático como aproximación inicial
    theta_sol = fzero(fun, theta_0, options);
    
    % Calcular la variación del ángulo (Steering angle real)
    delta_theta = theta_sol - theta_0;
    
    % Guardar en grados
    steering_angle_deg(i) = rad2deg(delta_theta);
end

% 4. Graficar Resultados
figure;
plot(stroke, steering_angle_deg, 'b-', 'LineWidth', 2);
grid on;
title('Simulación: Steering Angle vs. Rack Stroke');
xlabel('Rack Stroke (mm)');
ylabel('Steering Angle (Grados)');