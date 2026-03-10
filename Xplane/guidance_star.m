function [psi_ref, h_ref, v_ref, wp_idx, dist] = guidance_star(pos_n, pos_e, WPs, R_accept, wp_idx)
%GUIDANCE_STAR Guiagem LOS por waypoints (replica Guidance_Star do NL_guidance.slx).
%
%   [psi_ref, h_ref, v_ref, wp_idx, dist] = guidance_star(pos_n, pos_e, WPs, R_accept, wp_idx)
%
%   Inputs:
%     pos_n    - posicao Norte atual (m)
%     pos_e    - posicao Leste atual (m)
%     WPs      - matriz de waypoints [Norte, Leste, Alt, Vel] (Nx4)
%     R_accept - raio de aceitacao para troca de WP (m)
%     wp_idx   - indice do waypoint ativo (inicia em 2)
%
%   Outputs:
%     psi_ref  - proa desejada (rad)
%     h_ref    - altitude desejada (m)
%     v_ref    - velocidade desejada (m/s)
%     wp_idx   - indice atualizado (avanca se entrou no raio)
%     dist     - distancia horizontal ao WP ativo (m)

    num_wps = size(WPs, 1);

    % Distancia horizontal ao WP ativo
    delta_n = WPs(wp_idx, 1) - pos_n;
    delta_e = WPs(wp_idx, 2) - pos_e;
    dist = sqrt(delta_n^2 + delta_e^2);

    % Troca de waypoint se dentro do raio de aceitacao
    if dist <= R_accept && wp_idx < num_wps
        wp_idx = wp_idx + 1;
        % Recalcular para o novo WP
        delta_n = WPs(wp_idx, 1) - pos_n;
        delta_e = WPs(wp_idx, 2) - pos_e;
        dist = sqrt(delta_n^2 + delta_e^2);
    end

    % Proa desejada (LOS puro)
    psi_ref = atan2(delta_e, delta_n);

    % Altitude e velocidade do WP ativo
    h_ref = WPs(wp_idx, 3);
    v_ref = WPs(wp_idx, 4);
end
