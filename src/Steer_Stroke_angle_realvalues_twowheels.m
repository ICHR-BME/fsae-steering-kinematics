% =========================================================================
% Simulación: Ackermann Steering Angle vs Stroke
% =========================================================================
clear; clc; close all;

% --- 1. PARÁMETROS GEOMÉTRICOS ---
W_kp = 1101.21;     % Distancia transversal entre Kingpins (mm)
R_arm = 71.00;      % Longitud del steering arm (mm)
Y_rack = 40.00;     % Offset longitudinal de la cremallera (mm)
stroke_max = 31.75; % Recorrido máximo del rack (mm)

% Ángulo inicial: El diagrama marca 15.78° desde la vertical. 
% Lo pasamos al plano cartesiano restándolo de 90°.
theta_0_izq = deg2rad(90 - 15.78); 

% Coordenadas iniciales (Origen 0,0 en el Kingpin Izquierdo)
Bx_izq_0 = R_arm * cos(theta_0_izq);
By_izq_0 = R_arm * sin(theta_0_izq);

% El diagrama muestra una distancia en X de 441.96 mm para la bieleta
Rx_izq_0 = Bx_izq_0 + 441.96; 
Ry_0 = Y_rack;

% Calculamos la longitud real de la bieleta (L_tie) usando el teorema de Pitágoras
L_tie = sqrt((Rx_izq_0 - Bx_izq_0)^2 + (Ry_0 - By_izq_0)^2);

% Ángulo estático de la llanta derecha (Es un espejo de la izquierda)
theta_0_der = pi - theta_0_izq; 

% --- 2. VECTORES DE SIMULACIÓN ---
stroke = linspace(-stroke_max, stroke_max, 100); 
steer_izq = zeros(1, length(stroke));
steer_der = zeros(1, length(stroke));

options = optimset('Display', 'off'); 

% --- 3. SOLUCIÓN CINEMÁTICA ---
for i = 1:length(stroke)
    s = stroke(i); % Desplazamiento de la cremallera
    
    % Cinemática Rueda Izquierda
    Rx_izq = Rx_izq_0 + s;
    fun_izq = @(t) (R_arm*cos(t) - Rx_izq)^2 + (R_arm*sin(t) - Ry_0)^2 - L_tie^2;
    theta_sol_izq = fzero(fun_izq, theta_0_izq, options);
    steer_izq(i) = rad2deg(theta_sol_izq - theta_0_izq);
    
    % Cinemática Rueda Derecha
    Rx_der_0 = W_kp - Rx_izq_0;
    Rx_der = Rx_der_0 + s;
    fun_der = @(t) (W_kp + R_arm*cos(t) - Rx_der)^2 + (R_arm*sin(t) - Ry_0)^2 - L_tie^2;
    theta_sol_der = fzero(fun_der, theta_0_der, options);
    steer_der(i) = rad2deg(theta_sol_der - theta_0_der);
end

% --- 4. GRAFICAR RESULTADOS ---
figure('Name', 'Cinemática de Dirección FSAE', 'NumberTitle', 'off');
plot(stroke, steer_izq, 'b-', 'LineWidth', 2); hold on;
plot(stroke, steer_der, 'r-', 'LineWidth', 2);
grid on;
title('Geometría de Ackermann: Steering Angle vs. Rack Stroke');
xlabel('Rack Stroke (mm) [Negativo=Giro Izq, Positivo=Giro Der]');
ylabel('Steering Angle (Grados)');
legend('Rueda Izquierda', 'Rueda Derecha', 'Location', 'Best');

% --- 5. IMPRESIÓN DE VALIDACIÓN ---
fprintf('--- VALIDACIÓN CON DIAGRAMA FSAE ---\n');
fprintf('Ángulo Rueda Izq al tope (Stroke %.2f): %.2f°\n', stroke_max, abs(steer_izq(end)));
fprintf('Ángulo Rueda Der al tope (Stroke %.2f): %.2f°\n', stroke_max, abs(steer_der(end)));