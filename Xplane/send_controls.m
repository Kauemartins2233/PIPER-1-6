% function y = send_controls(u)
% % Esta função envia os comandos de controle para o X-Plane usando o XPC.
% % u(1) = elevator
% % u(2) = aileron
% % u(3) = rudder
% % u(4) = throttle
% 
% % Versão corrigida do vetor de controle com 6 elementos,
% % compatível com versões mais antigas do X-Plane.
% % Ordem: [elev, aileron, rudder, throttle, gear, flaps]
% ctrl = [u(1), u(2), u(3), u(4), 0, 0];
% 
% XPlaneConnect.sendCTRL(ctrl);
% 
% % O bloco exige uma saída, que não usamos.
% y = 0;
% 
% end

function status = send_controls(u)
    % Acessa a MESMA variável global do bloco de leitura
    global GlobalSocket;
    import XPlaneConnect.*;
    
    status = 0;

    % --- Lógica de Conexão Inteligente ---
    % Se o bloco de leitura ainda não abriu a conexão, este bloco abre.
    if isempty(GlobalSocket)
        try
            GlobalSocket = openUDP('127.0.0.1', 49009);
            disp('Conexão X-Plane aberta pelo bloco de ESCRITA.');
        catch
            % Segue sem travar
        end
    end
    
    % --- Envio ---
    % Só envia se o GlobalSocket for válido
    if ~isempty(GlobalSocket)
       % Verificação extra para garantir que é um objeto Java válido
       if isa(GlobalSocket, 'gov.nasa.xpc.XPlaneConnect')
           try
                elevator = u(1);
                aileron  = u(2);
                rudder   = u(3);
                throttle = u(4);
                
                ctrl_data = [elevator, aileron, rudder, throttle, -998, -998];
                
                sendCTRL(ctrl_data, 0, GlobalSocket);
                status = 1;
           catch
               % Se der erro crítico, limpa a global para tentar reconectar no próximo ciclo
               % GlobalSocket = []; 
           end
       end
    end
end