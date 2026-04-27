%% analise_sim.m - Analise pos-simulacao do NL_guidance
% Rodar APOS a simulacao terminar para verificar o comportamento
% dos controladores e identificar problemas.
%
% Uso: >> analise_sim

fprintf('\n========== ANALISE POS-SIMULACAO ==========\n\n');

%% ========== Localizar variavel ALT ==========
% O modelo pode salvar ALT diretamente no workspace OU dentro de 'out'
% (quando "Single simulation output" esta ativado no Model Settings).
ALT_data = [];

if exist('ALT', 'var')
    ALT_data = ALT;
    fprintf('  ALT encontrada no workspace (class=%s)\n', class(ALT));
elseif exist('out', 'var')
    % Tentar extrair ALT de dentro do objeto out
    try
        ALT_data = out.ALT;
        fprintf('  ALT extraida de out.ALT (class=%s)\n', class(ALT_data));
    catch
        % Listar o que tem dentro de out
        fprintf('  Variavel out existe mas out.ALT nao encontrada.\n');
        try
            who_out = out.who;
            fprintf('  Conteudo de out: %s\n', strjoin(who_out, ', '));
        catch
            fprintf('  (nao foi possivel listar conteudo de out)\n');
        end
    end
elseif exist('simOut', 'var')
    try
        ALT_data = simOut.ALT;
        fprintf('  ALT extraida de simOut.ALT (class=%s)\n', class(ALT_data));
    catch
        fprintf('  simOut existe mas simOut.ALT nao encontrada.\n');
    end
end

%% ========== Processar e plotar altitude ==========
if ~isempty(ALT_data)
    % Extrair tempo e dados conforme o formato
    if isa(ALT_data, 'timeseries')
        t_alt = ALT_data.Time;
        vals  = ALT_data.Data;
    elseif isstruct(ALT_data) && isfield(ALT_data, 'signals')
        t_alt = ALT_data.time;
        vals  = ALT_data.signals.values;
    elseif isstruct(ALT_data) && isfield(ALT_data, 'time')
        t_alt = ALT_data.time;
        vals  = ALT_data.data;
    else
        fprintf('  AVISO: formato de ALT nao reconhecido (class=%s).\n', class(ALT_data));
        t_alt = []; vals = [];
    end

    if ~isempty(t_alt)
        if size(vals, 2) >= 2
            h_ref_log = vals(:,1);
            alt_log   = vals(:,2);
        else
            h_ref_log = [];
            alt_log   = vals(:,1);
        end

        figure('Name', 'Analise Altitude');
        if ~isempty(h_ref_log)
            subplot(2,1,1);
            plot(t_alt, alt_log, 'b', t_alt, h_ref_log, 'r--', 'LineWidth', 1.5);
            legend('Altitude', 'h_{ref}'); grid on;
            title('Altitude vs Referencia');
            ylabel('Altitude (m)');

            subplot(2,1,2);
            plot(t_alt, h_ref_log - alt_log, 'k', 'LineWidth', 1.5);
            title('Erro de Altitude (h_{ref} - alt)');
            ylabel('Erro (m)'); xlabel('Tempo (s)'); grid on;
        else
            plot(t_alt, alt_log, 'b', 'LineWidth', 1.5);
            title('Altitude'); ylabel('Altitude (m)'); xlabel('Tempo (s)'); grid on;
        end

        fprintf('  Altitude: %.1f -> %.1f m (delta = %.1f m)\n', ...
            alt_log(1), alt_log(end), alt_log(end) - alt_log(1));
        if ~isempty(h_ref_log)
            fprintf('  h_ref:    %.1f -> %.1f m\n', h_ref_log(1), h_ref_log(end));
        end
    end
else
    fprintf('  AVISO: Variavel ALT nao encontrada.\n');
    fprintf('         Verifique: Model Settings > Data Import/Export >\n');
    fprintf('         desmarque "Single simulation output" OU use out.ALT\n');
end

%% ========== Verificar logsout ou ScopeData ==========
scope_vars = {'ScopeData', 'ScopeData1', 'ScopeData2', 'ScopeData3'};
for i = 1:length(scope_vars)
    if evalin('base', sprintf('exist(''%s'',''var'')', scope_vars{i}))
        fprintf('  Scope: %s encontrado no workspace\n', scope_vars{i});
    end
end

%% ========== Verificar trim ==========
fprintf('\n--- Verificacao de Trim ---\n');
Xp0 = dyn_rigidbody(0, Xe, Ue, par_gen, par_aero, par_prop);
trim_quality = norm([Xp0(1:9); Xp0(12)]);
fprintf('  trim_quality = %.6e\n', trim_quality);
fprintf('  up=%.4e vp=%.4e wp=%.4e\n', Xp0(1), Xp0(2), Xp0(3));
fprintf('  pp=%.4e qp=%.4e rp=%.4e\n', Xp0(4), Xp0(5), Xp0(6));
fprintf('  xNp=%.4f xEp=%.4f xDp=%.4f\n', Xp0(10), Xp0(11), Xp0(12));

if abs(Xp0(12)) > 0.01
    fprintf('\n  ATENCAO: xDp = %.4f m/s no trim!\n', Xp0(12));
    fprintf('  Em 180s, a altitude desvia %+.1f m.\n', -Xp0(12)*180);
end

fprintf('\n========== FIM DA ANALISE ==========\n');
