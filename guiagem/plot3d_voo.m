%% plot3d_voo.m - Plota trajetoria 3D da aeronave
% Rodar APOS a simulacao terminar.
% Requer variavel POS no workspace (To Workspace com [xN, xE, alt]).
%
% Se POS nao existir, tenta extrair de 'out' ou dos estados.
%
% Uso: >> plot3d_voo

fprintf('\n========== PLOT 3D DO VOO ==========\n');

%% ========== Localizar dados de posicao ==========
pos_data = [];
t_pos = [];

% Tentar POS diretamente
if exist('POS', 'var')
    pos_data = POS;
    fprintf('  POS encontrada no workspace\n');
elseif exist('out', 'var')
    % Tentar out.POS
    try
        pos_data = out.POS;
        fprintf('  POS extraida de out.POS\n');
    catch
    end

    % Tentar out.yout (saidas do modelo)
    if isempty(pos_data)
        try
            yout = out.yout;
            fprintf('  yout encontrada em out\n');
            if isa(yout, 'timeseries')
                t_pos = yout.Time;
                % S-function outputs: indices 10,11,12 = xN, xE, -xD(=alt)
                if size(yout.Data, 2) >= 12
                    pos_data = yout.Data(:, 10:12);
                    fprintf('  Posicao extraida de yout (cols 10-12)\n');
                end
            end
        catch
        end
    end

    % Tentar out.xout (estados do modelo)
    if isempty(pos_data)
        try
            xout_raw = out.xout;
            fprintf('  xout encontrada em out (class=%s)\n', class(xout_raw));

            if isnumeric(xout_raw) && size(xout_raw, 2) >= 12
                % Formato matriz (Simulink classico)
                t_pos = out.tout;
                xN  = xout_raw(:, 10);
                xE  = xout_raw(:, 11);
                alt = -xout_raw(:, 12);
                pos_data = [xN, xE, alt];
                fprintf('  Posicao extraida de xout matriz (cols 10-12)\n');

            elseif isa(xout_raw, 'Simulink.SimulationData.Dataset')
                % Formato Dataset (Simulink moderno)
                fprintf('  xout eh Dataset com %d elementos\n', xout_raw.numElements);
                % Concatenar todos os estados
                all_states = [];
                t_pos = [];
                for k = 1:xout_raw.numElements
                    el = xout_raw.getElement(k);
                    if isa(el, 'timeseries')
                        if isempty(t_pos); t_pos = el.Time; end
                        all_states = [all_states, el.Data];
                    end
                end
                if size(all_states, 2) >= 12
                    xN  = all_states(:, 10);
                    xE  = all_states(:, 11);
                    alt = -all_states(:, 12);
                    pos_data = [xN, xE, alt];
                    fprintf('  Posicao extraida de Dataset (cols 10-12)\n');
                else
                    fprintf('  Dataset tem apenas %d colunas\n', size(all_states, 2));
                end

            elseif isa(xout_raw, 'timeseries')
                t_pos = xout_raw.Time;
                xdata = xout_raw.Data;
                if size(xdata, 2) >= 12
                    xN  = xdata(:, 10);
                    xE  = xdata(:, 11);
                    alt = -xdata(:, 12);
                    pos_data = [xN, xE, alt];
                    fprintf('  Posicao extraida de xout timeseries\n');
                end
            end
        catch ME
            fprintf('  Erro ao processar xout: %s\n', ME.message);
        end
    end

    % Tentar logsout
    if isempty(pos_data)
        try
            logsout = out.logsout;
            fprintf('  logsout encontrada. Sinais disponiveis:\n');
            for k = 1:logsout.numElements
                el = logsout.getElement(k);
                fprintf('    [%d] %s\n', k, el.Name);
            end
        catch
        end
    end
end

%% ========== Extrair xN, xE, alt ==========
if ~isempty(pos_data)
    if isa(pos_data, 'timeseries')
        t_pos = pos_data.Time;
        vals  = pos_data.Data;
    elseif isstruct(pos_data) && isfield(pos_data, 'signals')
        t_pos = pos_data.time;
        vals  = pos_data.signals.values;
    elseif isnumeric(pos_data)
        vals = pos_data;
        % t_pos ja definido acima
    else
        fprintf('  Formato de POS nao reconhecido (class=%s)\n', class(pos_data));
        vals = [];
    end

    if ~isempty(vals) && size(vals, 2) >= 3
        xN  = vals(:,1);
        xE  = vals(:,2);
        alt = vals(:,3);
    elseif ~isempty(vals) && size(vals, 2) == 2
        xN  = vals(:,1);
        xE  = vals(:,2);
        alt = ones(size(xN)) * 560;  % Usar altitude nominal
        fprintf('  AVISO: Apenas 2 colunas — usando altitude = 560m\n');
    else
        fprintf('  ERRO: Dados de posicao insuficientes.\n');
        return;
    end
else
    fprintf('\n  Dados de posicao NAO encontrados.\n');
    fprintf('  Para habilitar, adicione um To Workspace no modelo:\n');
    fprintf('    1) No Autopilot1, conecte um Mux com [xN, xE, alt]\n');
    fprintf('       (sinais do Demux5, outputs 1, 2 e 3)\n');
    fprintf('    2) Conecte o Mux a um bloco To Workspace\n');
    fprintf('    3) Variable name: POS, Save format: Timeseries\n');
    fprintf('    4) Re-rode a simulacao e execute plot3d_voo\n');
    fprintf('\n  Alternativa rapida: habilite "Save states" nas configs do modelo:\n');
    fprintf('    Model Settings > Data Import/Export > marque "States"\n');
    fprintf('    Re-rode e execute plot3d_voo\n');
    return;
end

%% ========== Plot 3D ==========
figure('Name', 'Trajetoria 3D', 'Position', [100 100 900 700]);

% --- Subplot 1: Vista 3D ---
subplot(2,2,[1 3]);
plot3(xE, xN, alt, 'b-', 'LineWidth', 1.5);
hold on;

% Waypoints (apenas markers, sem linha conectando)
if exist('WPs', 'var')
    plot3(WPs(:,2), WPs(:,1), WPs(:,3), 'rs', 'MarkerSize', 10, ...
          'MarkerFaceColor', 'r');
    % Numerar waypoints
    for i = 1:size(WPs, 1)
        text(WPs(i,2)+5, WPs(i,1)+5, WPs(i,3)+2, sprintf('WP%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
    end
end

% Marcar inicio e fim
plot3(xE(1), xN(1), alt(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(xE(end), xN(end), alt(end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

xlabel('Leste (m)'); ylabel('Norte (m)'); zlabel('Altitude (m)');
title('Trajetoria 3D da Aeronave');
legend('Trajetoria', 'Waypoints', 'Inicio', 'Fim', 'Location', 'best');
grid on; axis equal;
view(30, 25);
hold off;

% --- Subplot 2: Vista de cima (Norte x Leste) ---
subplot(2,2,2);
plot(xE, xN, 'b-', 'LineWidth', 1.5);
hold on;
if exist('WPs', 'var')
    plot(WPs(:,2), WPs(:,1), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    for i = 1:size(WPs, 1)
        text(WPs(i,2)+5, WPs(i,1)+5, sprintf('WP%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
    end
    % Circulo de aceitacao
    if exist('R_accept', 'var')
        th = linspace(0, 2*pi, 100);
        for i = 1:size(WPs, 1)
            plot(WPs(i,2) + R_accept*cos(th), WPs(i,1) + R_accept*sin(th), ...
                 'r--', 'LineWidth', 0.5);
        end
    end
end
plot(xE(1), xN(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(xE(end), xN(end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('Leste (m)'); ylabel('Norte (m)');
title('Vista Superior (Ground Track)');
grid on; axis equal;
hold off;

% --- Subplot 3: Altitude vs tempo ---
subplot(2,2,4);
if ~isempty(t_pos)
    plot(t_pos, alt, 'b-', 'LineWidth', 1.5);
    xlabel('Tempo (s)');
else
    plot(alt, 'b-', 'LineWidth', 1.5);
    xlabel('Amostra');
end
ylabel('Altitude (m)');
title('Altitude ao Longo do Voo');
grid on;

%% ========== Estatisticas ==========
fprintf('\n--- Estatisticas do Voo ---\n');
fprintf('  Posicao inicial: N=%.1f  E=%.1f  Alt=%.1f m\n', xN(1), xE(1), alt(1));
fprintf('  Posicao final:   N=%.1f  E=%.1f  Alt=%.1f m\n', xN(end), xE(end), alt(end));
fprintf('  Altitude: min=%.2f  max=%.2f  media=%.2f m\n', min(alt), max(alt), mean(alt));
fprintf('  Distancia total percorrida: %.1f m\n', ...
    sum(sqrt(diff(xN).^2 + diff(xE).^2 + diff(alt).^2)));

if exist('WPs', 'var')
    % Distancia ao ultimo WP
    d_final = sqrt((xN(end)-WPs(end,1))^2 + (xE(end)-WPs(end,2))^2);
    fprintf('  Distancia ao WP final: %.1f m\n', d_final);
end

fprintf('\n========== FIM DO PLOT 3D ==========\n');
