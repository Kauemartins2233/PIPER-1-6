function gui_waypoints_HIL()
%% gui_waypoints_HIL - Versao HIL da GUI de waypoints.
% Identica a guiagem/gui_waypoints.m, mas o botao SIMULAR roda
% Arduino/guiagem embarcada/modelo_HIL_guiagem.slx (controle + guiagem
% rodam no Arduino Mega via COM3) em vez do NL_guidance.slx puro.
%
% Uso:
%   >> gui_waypoints_HIL
%
% Pre-req: Mega gravado com arduino_guiagem_controle.ino, COM3 livre.

    %% ========== Inicializacao ==========
    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));  % raiz do projeto
    hilDir  = fileparts(mfilename('fullpath'));                        % .../guiagem embarcada

    oldDir = pwd;
    cd(rootDir);
    evalin('base', 'inicializar');     % limpa COM3 + carrega ganhos
    cd(oldDir);

    addpath(hilDir);
    addpath(fullfile(hilDir, 'tools'));

    par_aero  = evalin('base', 'par_aero');
    par_prop  = evalin('base', 'par_prop');
    par_gen   = evalin('base', 'par_gen');
    Xe        = evalin('base', 'Xe');
    Ue        = evalin('base', 'Ue');
    Xe_init   = evalin('base', 'Xe_init');
    h_eq      = evalin('base', 'h_eq');

    %% ========== Dados ==========
    wp_data = [0, 0, h_eq, 15];

    %% ========== Figura ==========
    fig = uifigure('Name', 'Waypoints HIL - Piper J-3 Cub 1/6 (Arduino Mega)', ...
        'Position', [100 100 1000 650], ...
        'Color', [0.95 0.95 0.95]);

    ax = uiaxes(fig, 'Position', [20 20 580 600]);
    ax.XLabel.String = 'Leste (m)';
    ax.YLabel.String = 'Norte (m)';
    ax.Title.String  = 'Clique para adicionar waypoints';
    ax.XGrid = 'on'; ax.YGrid = 'on'; ax.Box = 'on'; ax.FontSize = 11;
    hold(ax, 'on');
    ax.ButtonDownFcn = @mapClick;

    panelX = 620; panelW = 360;

    uilabel(fig, 'Position', [panelX 600 panelW 30], ...
        'Text', 'Waypoints (HIL Mega)', ...
        'FontSize', 16, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');

    tbl = uitable(fig, 'Position', [panelX 360 panelW 235], ...
        'ColumnName', {'#', 'Norte (m)', 'Leste (m)', 'Alt (m)', 'Vel (m/s)'}, ...
        'ColumnWidth', {30, 75, 75, 70, 70}, ...
        'ColumnEditable', [false true true true true], ...
        'CellEditCallback', @tableEdited);

    uibutton(fig, 'Position', [panelX 320 170 30], ...
        'Text', 'Remover Ultimo', 'ButtonPushedFcn', @removeLastWP);
    uibutton(fig, 'Position', [panelX+180 320 170 30], ...
        'Text', 'Limpar Tudo', 'ButtonPushedFcn', @clearWPs);

    yPos = 270;
    uilabel(fig, 'Position', [panelX yPos 130 22], ...
        'Text', 'Altitude do proximo WP:', 'FontSize', 11);
    fldAlt = uieditfield(fig, 'numeric', ...
        'Position', [panelX+170 yPos 80 22], ...
        'Value', h_eq, 'Limits', [0 1000]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 'm');

    yPos = yPos - 35;
    uilabel(fig, 'Position', [panelX yPos 160 22], ...
        'Text', 'Velocidade do proximo WP:', 'FontSize', 11);
    fldVel = uieditfield(fig, 'numeric', ...
        'Position', [panelX+170 yPos 80 22], ...
        'Value', 15, 'Limits', [5 50]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 'm/s');

    yPos = yPos - 35;
    uilabel(fig, 'Position', [panelX yPos 160 22], ...
        'Text', 'Tempo de simulacao:', 'FontSize', 11);
    fldStopTime = uieditfield(fig, 'numeric', ...
        'Position', [panelX+170 yPos 80 22], ...
        'Value', 200, 'Limits', [10 5000]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 's');

    uibutton(fig, 'Position', [panelX 100 panelW 50], ...
        'Text', 'SIMULAR (HIL)', ...
        'FontSize', 16, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.4 0.7], 'FontColor', 'white', ...
        'ButtonPushedFcn', @runSimulation);

    lblStatus = uilabel(fig, 'Position', [panelX 70 panelW 40], ...
        'Text', 'Conecte o Arduino em COM3 antes de simular.', ...
        'FontSize', 10, 'WordWrap', 'on', 'FontColor', [0.3 0.3 0.3]);

    uilabel(fig, 'Position', [panelX 20 panelW 45], ...
        'Text', 'Max 32 waypoints. R_accept e fixo no firmware (80 m). Heading hold K=0.3.', ...
        'FontSize', 9, 'WordWrap', 'on', 'FontColor', [0.5 0.5 0.5]);

    updateTable(); updateMap();

    %% ==================== CALLBACKS ====================

    function mapClick(~, event)
        pt = event.IntersectionPoint;
        leste = pt(1); norte = pt(2);
        wp_data(end+1, :) = [norte, leste, fldAlt.Value, fldVel.Value];
        updateTable(); updateMap();
        lblStatus.Text = sprintf('WP%d: N=%.0f E=%.0f Alt=%.0f V=%.0f', ...
            size(wp_data,1), norte, leste, fldAlt.Value, fldVel.Value);
    end

    function removeLastWP(~, ~)
        if size(wp_data,1) > 1
            wp_data(end, :) = [];
            updateTable(); updateMap();
        end
    end

    function clearWPs(~, ~)
        wp_data = [0, 0, h_eq, 15];
        updateTable(); updateMap();
    end

    function tableEdited(~, event)
        row = event.Indices(1); col = event.Indices(2);
        if col >= 2 && col <= 5
            wp_data(row, col-1) = event.NewData;
            updateMap();
        end
    end

    function [bad_idx, msgs] = check_wps()
        n = size(wp_data, 1);
        bad = false(n, 1);
        msgs = {};
        for i = 2:n-1
            v1 = wp_data(i,   1:2) - wp_data(i-1, 1:2);
            v2 = wp_data(i+1, 1:2) - wp_data(i,   1:2);
            d2 = norm(v2);
            if d2 < 1e-6, continue; end
            ang = atan2d(v1(1)*v2(2)-v1(2)*v2(1), v1(1)*v2(1)+v1(2)*v2(2));
            abs_ang = abs(ang);
            if abs_ang < 30
                req = 100;
            elseif abs_ang < 90
                req = 200;
            elseif abs_ang < 135
                req = 300;
            else
                req = 400;
            end
            if d2 < req
                bad(i)   = true;
                bad(i+1) = true;
                msgs{end+1} = sprintf(...
                    'WP%d->WP%d: curva %.0f deg, dist %.0f m (minimo ~%d m)', ...
                    i, i+1, abs_ang, d2, req); %#ok<AGROW>
            end
        end
        bad_idx = find(bad);
    end

    function updateTable()
        n = size(wp_data,1);
        tbl.Data = [num2cell((1:n)'), num2cell(wp_data)];
        % Limpa estilos antigos e aplica vermelho+negrito nos WPs problematicos
        try
            removeStyle(tbl);
        catch
        end
        bad_idx = check_wps();
        if ~isempty(bad_idx)
            try
                s = uistyle('FontColor', [0.8 0 0], 'FontWeight', 'bold');
                addStyle(tbl, s, 'row', bad_idx);
            catch
            end
        end
    end

    function updateMap()
        cla(ax); hold(ax,'on');
        n = size(wp_data,1); R = 80; th = linspace(0,2*pi,100);
        if n > 1
            plot(ax, wp_data(:,2), wp_data(:,1), '--', 'Color',[0.5 0.5 0.5]);
        end
        for i = 1:n
            plot(ax, wp_data(i,2)+R*cos(th), wp_data(i,1)+R*sin(th), 'r--');
            plot(ax, wp_data(i,2), wp_data(i,1), 'rs', 'MarkerSize',10, 'MarkerFaceColor','r');
            text(ax, wp_data(i,2)+R*0.3, wp_data(i,1)+R*0.3, ...
                sprintf('WP%d\n%.0fm', i, wp_data(i,3)), ...
                'FontSize',9, 'FontWeight','bold', 'Color','r');
        end
        plot(ax, 0, 0, 'go', 'MarkerSize',12, 'MarkerFaceColor','g');
        if n > 1
            m = max(R*2, 100);
            ax.XLim = [min(wp_data(:,2))-m, max(wp_data(:,2))+m];
            ax.YLim = [min(wp_data(:,1))-m, max(wp_data(:,1))+m];
        else
            ax.XLim = [-200 200]; ax.YLim = [-200 200];
        end
        ax.Title.String = sprintf('Waypoints (%d pontos) - HIL Mega', n);
        ax.ButtonDownFcn = @mapClick;
        hold(ax,'off');
    end

    function runSimulation(~, ~)
        if size(wp_data,1) < 2
            lblStatus.Text = 'Adicione pelo menos 2 waypoints.';
            lblStatus.FontColor = [0.8 0 0]; return;
        end
        if size(wp_data,1) > 32
            lblStatus.Text = 'Max 32 waypoints (limite do firmware).';
            lblStatus.FontColor = [0.8 0 0]; return;
        end

        % Aviso de WPs problematicos (curvas bruscas/distancia insuficiente)
        [bad_idx, msgs] = check_wps();
        if ~isempty(bad_idx)
            txt = sprintf(['Foram detectadas curvas problematicas que ' ...
                'podem causar overshoot, oscilacao ou stall:\n\n%s\n\n' ...
                'A trajetoria provavelmente nao seguira os waypoints ' ...
                'fielmente. Continuar mesmo assim?'], strjoin(msgs, sprintf('\n')));
            sel = uiconfirm(fig, txt, 'Aviso: waypoints problematicos', ...
                'Options', {'Continuar', 'Cancelar'}, ...
                'DefaultOption', 2, 'CancelOption', 2, ...
                'Icon', 'warning');
            if strcmp(sel, 'Cancelar')
                lblStatus.Text = 'Simulacao cancelada pelo usuario.';
                lblStatus.FontColor = [0.5 0.5 0.5];
                return;
            end
        end

        lblStatus.Text = 'Enviando WPs e simulando HIL...';
        lblStatus.FontColor = [0 0 0.6]; drawnow;

        try
            % Re-publica vars no base (caso usuario tenha mexido)
            assignin('base','par_aero',par_aero);
            assignin('base','par_prop',par_prop);
            assignin('base','par_gen', par_gen);
            assignin('base','Xe',      Xe);
            assignin('base','Ue',      Ue);
            assignin('base','Xe_init', Xe_init);

            % Interpolacao em altitude
            max_dalt = 30;
            wp_interp = wp_data(1,:);
            for i = 2:size(wp_data,1)
                dalt = abs(wp_data(i,3) - wp_data(i-1,3));
                if dalt > max_dalt
                    n_sub = ceil(dalt/max_dalt);
                    for k = 1:n_sub-1
                        frac = k/n_sub;
                        wp_interp(end+1,:) = wp_data(i-1,:) + frac*(wp_data(i,:)-wp_data(i-1,:));
                    end
                end
                wp_interp(end+1,:) = wp_data(i,:);
            end
            if size(wp_interp,1) > 32
                error('Apos interpolacao deu %d WPs (>32). Use menos pontos ou variacoes menores.', ...
                      size(wp_interp,1));
            end
            assignin('base','WPs', wp_interp);

            % Limpa serial pra forcar re-upload de WPs
            try, evalin('base','clear hil_serial_step_guiagem'); catch, end
            try
                sp_tmp = serialportfind;
                if ~isempty(sp_tmp), delete(sp_tmp); end
            catch
            end

            mdl = 'modelo_HIL_guiagem';
            modelPath = fullfile(hilDir, [mdl '.slx']);
            if ~exist(modelPath,'file')
                error('Modelo nao encontrado: %s. Rode criar_modelo_HIL_guiagem primeiro.', modelPath);
            end
            load_system(modelPath);
            set_param(mdl, 'StopTime', num2str(fldStopTime.Value));

            out = sim(mdl);
            assignin('base','out', out);
            assignin('base','WPs', wp_data);
            assignin('base','R_accept', 80);

            evalin('base','plot3d_voo');

            lblStatus.Text = 'HIL concluido com sucesso!';
            lblStatus.FontColor = [0 0.5 0];
        catch ME
            lblStatus.Text = sprintf('Erro: %s', ME.message);
            lblStatus.FontColor = [0.8 0 0];
            fprintf('Erro HIL:\n%s\n', getReport(ME));
        end
    end
end
