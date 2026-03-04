function gui_waypoints()
%% gui_waypoints - Interface visual para inserção de waypoints
% Permite clicar no mapa para posicionar waypoints, definir altitude e
% velocidade, e simular automaticamente o modelo NL_guidance.slx.
%
% Uso:
%   >> gui_waypoints
%
% A GUI carrega automaticamente todos os parâmetros necessários.
% Não é preciso rodar inicializar.m antes.

    %% ========== Inicialização ==========
    rootDir = fileparts(fileparts(mfilename('fullpath')));  % raiz do projeto
    addpath(fullfile(rootDir, 'modelos', 'Não Linear'));
    addpath(fullfile(rootDir, 'guiagem'));

    % Carregar parâmetros da aeronave
    matFile = fullfile(rootDir, 'modelos', 'Não Linear', 'Sato_longitudinal_Piper_1_6.mat');
    dados = load(matFile);
    par_aero = dados.par_aero;
    par_prop = dados.par_prop;
    par_gen  = dados.par_gen;

    % Estado de equilíbrio
    equilibrium;  % Define Xe (12x1) e Ue (4x1)
    h_eq = -Xe(12);  % altitude de equilíbrio = 100m

    % Ganhos PID (mesmos do inicializar.m)
    C_alt.Kp = 0.596304245000559;
    C_alt.Ki = 0.356254495459697;
    C_alt.Kd = -0.0141697733728916;
    C_alt.N  = 6.17157424867561;

    C_theta.Kp = 20.3142831421082;
    C_theta.Ki = 22.5973845921282;
    C_theta.Kd = 1.76724278063426;
    C_theta.N  = 1159.39165466191;

    C_vel.Kp = 0.0786752433250596;
    C_vel.Ki = 0.0200000000000000;
    C_vel.Kd = 0.0151687829718727;
    C_vel.N  = 77.0155336495376;

    C_phi.Kp = 26.7874562402529;
    C_phi.Ki = 13.1675046432808;
    C_phi.Kd = -0.0875715773472267;
    C_phi.N  = 305.892129064194;

    Kq = 0.1;
    Kp_gain = 0.119;  % SAS rolamento
    Kr = 0.15;

    % Compatibilidade
    INPUTS    = Ue';
    TrimInput = Ue';
    Kp_sas    = Kp_gain;
    Xe_init   = Xe;

    %% ========== Dados dos Waypoints ==========
    % WP1 fixo na posição inicial (0, 0, h_eq, 15)
    wp_data = [0, 0, h_eq, 15];  % matriz Nx4

    %% ========== Criar Figura ==========
    fig = uifigure('Name', 'Waypoints - Piper J-3 Cub 1/6', ...
        'Position', [100 100 1000 650], ...
        'Color', [0.95 0.95 0.95]);

    %% ========== Mapa 2D (UIAxes) ==========
    ax = uiaxes(fig, 'Position', [20 20 580 600]);
    ax.XLabel.String = 'Leste (m)';
    ax.YLabel.String = 'Norte (m)';
    ax.Title.String  = 'Clique para adicionar waypoints';
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    ax.Box = 'on';
    ax.FontSize = 11;
    hold(ax, 'on');

    % Callback de clique no mapa
    ax.ButtonDownFcn = @mapClick;

    %% ========== Painel Lateral ==========
    panelX = 620;
    panelW = 360;

    % Título
    uilabel(fig, 'Position', [panelX 600 panelW 30], ...
        'Text', 'Waypoints', ...
        'FontSize', 16, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');

    % Tabela de waypoints
    tbl = uitable(fig, 'Position', [panelX 360 panelW 235], ...
        'ColumnName', {'#', 'Norte (m)', 'Leste (m)', 'Alt (m)', 'Vel (m/s)'}, ...
        'ColumnWidth', {30, 75, 75, 70, 70}, ...
        'ColumnEditable', [false true true true true], ...
        'CellEditCallback', @tableEdited);

    % --- Botões de controle ---
    btnRemover = uibutton(fig, 'Position', [panelX 320 170 30], ...
        'Text', 'Remover Último', ...
        'ButtonPushedFcn', @removeLastWP);

    btnLimpar = uibutton(fig, 'Position', [panelX+180 320 170 30], ...
        'Text', 'Limpar Tudo', ...
        'ButtonPushedFcn', @clearWPs);

    % --- Campos de entrada ---
    yPos = 270;

    uilabel(fig, 'Position', [panelX yPos 130 22], ...
        'Text', 'Altitude do próximo WP:', 'FontSize', 11);
    fldAlt = uieditfield(fig, 'numeric', ...
        'Position', [panelX+170 yPos 80 22], ...
        'Value', h_eq, ...
        'Limits', [0 1000]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 'm');

    yPos = yPos - 35;
    uilabel(fig, 'Position', [panelX yPos 160 22], ...
        'Text', 'Velocidade do próximo WP:', 'FontSize', 11);
    fldVel = uieditfield(fig, 'numeric', ...
        'Position', [panelX+170 yPos 80 22], ...
        'Value', 15, ...
        'Limits', [5 50]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 'm/s');

    yPos = yPos - 35;
    uilabel(fig, 'Position', [panelX yPos 160 22], ...
        'Text', 'Raio de aceitação:', 'FontSize', 11);
    fldRaccept = uieditfield(fig, 'numeric', ...
        'Position', [panelX+170 yPos 80 22], ...
        'Value', 80, ...
        'Limits', [10 500]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 'm');

    % --- Botão SIMULAR ---
    btnSimular = uibutton(fig, 'Position', [panelX 120 panelW 50], ...
        'Text', 'SIMULAR', ...
        'FontSize', 16, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.2], ...
        'FontColor', 'white', ...
        'ButtonPushedFcn', @runSimulation);

    % --- Label de status ---
    lblStatus = uilabel(fig, 'Position', [panelX 70 panelW 40], ...
        'Text', 'Pronto. Clique no mapa para adicionar waypoints.', ...
        'FontSize', 10, 'WordWrap', 'on', ...
        'FontColor', [0.3 0.3 0.3]);

    % --- Info ---
    uilabel(fig, 'Position', [panelX 20 panelW 45], ...
        'Text', 'WP1 é fixo na origem (posição inicial). A guiagem inicia mirando WP2.', ...
        'FontSize', 9, 'WordWrap', 'on', ...
        'FontColor', [0.5 0.5 0.5]);

    %% ========== Desenho inicial ==========
    updateTable();
    updateMap();

    %% ==================== CALLBACKS ====================

    function mapClick(~, event)
        % Adicionar waypoint na posição clicada
        pt = event.IntersectionPoint;
        leste = pt(1);
        norte = pt(2);
        alt = fldAlt.Value;
        vel = fldVel.Value;

        wp_data(end+1, :) = [norte, leste, alt, vel];
        updateTable();
        updateMap();
        lblStatus.Text = sprintf('WP%d adicionado: N=%.0f  E=%.0f  Alt=%.0f  V=%.0f', ...
            size(wp_data, 1), norte, leste, alt, vel);
    end

    function removeLastWP(~, ~)
        if size(wp_data, 1) > 1
            wp_data(end, :) = [];
            updateTable();
            updateMap();
            lblStatus.Text = 'Último waypoint removido.';
        else
            lblStatus.Text = 'WP1 é fixo e não pode ser removido.';
        end
    end

    function clearWPs(~, ~)
        wp_data = [0, 0, h_eq, 15];
        updateTable();
        updateMap();
        lblStatus.Text = 'Waypoints resetados. Apenas WP1 (origem).';
    end

    function tableEdited(~, event)
        row = event.Indices(1);
        col = event.Indices(2);
        % Coluna 1 é #, colunas 2-5 são Norte, Leste, Alt, Vel
        if col >= 2 && col <= 5
            wp_data(row, col-1) = event.NewData;
            updateMap();
        end
    end

    function updateTable()
        n = size(wp_data, 1);
        nums = (1:n)';
        tbl.Data = [num2cell(nums), num2cell(wp_data)];
    end

    function updateMap()
        cla(ax);
        hold(ax, 'on');

        n = size(wp_data, 1);
        R = fldRaccept.Value;
        theta_circ = linspace(0, 2*pi, 100);

        % Linhas conectando waypoints
        if n > 1
            plot(ax, wp_data(:,2), wp_data(:,1), '--', ...
                'Color', [0.5 0.5 0.5], 'LineWidth', 1);
        end

        % Waypoints e círculos de aceitação
        for i = 1:n
            norte_i = wp_data(i, 1);
            leste_i = wp_data(i, 2);

            % Círculo de aceitação
            plot(ax, leste_i + R*cos(theta_circ), norte_i + R*sin(theta_circ), ...
                'r--', 'LineWidth', 0.5);

            % Marcador
            plot(ax, leste_i, norte_i, 'rs', 'MarkerSize', 10, ...
                'MarkerFaceColor', 'r');

            % Rótulo
            text(ax, leste_i + R*0.3, norte_i + R*0.3, ...
                sprintf('WP%d\n%.0fm', i, wp_data(i,3)), ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
        end

        % Marcar origem
        plot(ax, 0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

        % Ajustar eixos
        if n > 1
            margem = max(R * 2, 100);
            xlims = [min(wp_data(:,2)) - margem, max(wp_data(:,2)) + margem];
            ylims = [min(wp_data(:,1)) - margem, max(wp_data(:,1)) + margem];
            ax.XLim = xlims;
            ax.YLim = ylims;
        else
            ax.XLim = [-200 200];
            ax.YLim = [-200 200];
        end

        ax.XLabel.String = 'Leste (m)';
        ax.YLabel.String = 'Norte (m)';
        ax.Title.String  = sprintf('Mapa de Waypoints (%d pontos)', n);

        % Habilitar clique (precisa re-setar após cla)
        ax.ButtonDownFcn = @mapClick;
        hold(ax, 'off');
    end

    function runSimulation(~, ~)
        % Verificar mínimo de waypoints
        if size(wp_data, 1) < 2
            lblStatus.Text = 'Adicione pelo menos 2 waypoints para simular.';
            lblStatus.FontColor = [0.8 0 0];
            return;
        end

        lblStatus.Text = 'Simulando...';
        lblStatus.FontColor = [0 0 0.6];
        drawnow;

        try
            R_accept_val = fldRaccept.Value;

            % Atribuir todas as variáveis no workspace base
            assignin('base', 'par_aero', par_aero);
            assignin('base', 'par_prop', par_prop);
            assignin('base', 'par_gen',  par_gen);
            assignin('base', 'Xe',       Xe);
            assignin('base', 'Ue',       Ue);
            assignin('base', 'Xe_init',  Xe_init);

            % Ganhos PID
            assignin('base', 'C_alt',   C_alt);
            assignin('base', 'C_theta', C_theta);
            assignin('base', 'C_vel',   C_vel);
            assignin('base', 'C_phi',   C_phi);
            assignin('base', 'Kq',      Kq);
            assignin('base', 'Kp',      Kp_gain);
            assignin('base', 'Kr',      Kr);

            % Compatibilidade
            assignin('base', 'INPUTS',    INPUTS);
            assignin('base', 'TrimInput', TrimInput);
            assignin('base', 'Kp_sas',   Kp_sas);

            % Waypoints e raio
            assignin('base', 'WPs',      wp_data);
            assignin('base', 'R_accept', R_accept_val);

            % Carregar modelo e simular
            modelPath = fullfile(rootDir, 'guiagem', 'NL_guidance.slx');
            load_system(modelPath);
            out = sim('NL_guidance');
            assignin('base', 'out', out);
            assignin('base', 'WPs', wp_data);
            assignin('base', 'R_accept', R_accept_val);

            % Plotar resultados
            evalin('base', 'plot3d_voo');

            lblStatus.Text = 'Simulação concluída com sucesso!';
            lblStatus.FontColor = [0 0.5 0];

        catch ME
            lblStatus.Text = sprintf('Erro: %s', ME.message);
            lblStatus.FontColor = [0.8 0 0];
            fprintf('Erro na simulação:\n%s\n', getReport(ME));
        end
    end

end
