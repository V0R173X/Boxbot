function serialReadCallback(src, ~)
    % Máquina de estados para recepção de pacotes
    persistent state buffer
    global dados_recebidos

    if isempty(state)
        state = "WAIT_START";
        buffer = [];
    end

    try
        bytes = read(src, src.NumBytesAvailable, "uint8");

        for b = bytes'
            switch state
                case "WAIT_START"
                    if b == 0xAA % Encontrou o início!
                        buffer = b;
                        state = "CMD";
                    end
                    % Se não for 0xAA (ex: um log 'E', 'S', 'P'...),
                    % ele simplesmente ignora e continua no estado WAIT_START.
                    % Isso está PERFEITO.

                case "CMD"
                    buffer(end+1) = b;
                    state = "ID";
                case "ID"
                    buffer(end+1) = b;
                    state = "VAL0";
                case "VAL0"
                    buffer(end+1) = b;
                    state = "VAL1";
                case "VAL1"
                    buffer(end+1) = b;
                    state = "CHECK";
                case "CHECK"
                    buffer(end+1) = b;
                    state = "WAIT_END";

                case "WAIT_END"
                    if b == 0xBB
                        buffer(end+1) = b; % Frame completo

                        % ---- Verificação de checksum CORRIGIDA ----
                        cmd   = buffer(2);
                        id    = buffer(3);
                        val0  = buffer(4);
                        val1  = buffer(5);
                        check = buffer(6);

                        % * A CORREÇÃO ESTÁ AQUI *
                        % Converte para 'double' ANTES de somar
                        soma_dados = double(cmd) + double(id) + double(val0) + double(val1);
                        
                        % 'mod' faz o "dar a volta" (wrap-around) igual ao C
                        soma_LSB = uint8(mod(soma_dados, 256));

                        if check == soma_LSB
                            % Check OK → pacote válido
                            valor_frame = typecast(buffer(4:5), 'uint16');
                            dados_recebidos = sprintf('CMD: %c, ID: %c, Valor: %d (Hex: %s)', ...
                                                      char(cmd), char(id), valor_frame, dec2hex(buffer));
                        else
                            warning("Checksum inválido (Recebido: %02X, Calculado: %02X). Frame: %s", ...
                                    check, soma_LSB, dec2hex(buffer));
                        end
                    else
                        % Erro: fim incorreto → reset
                        warning("Frame mal formatado: EOF (0xBB) esperado.");
                    end
                    
                    % Reseta a FSM para caçar o próximo 0xAA
                    state = "WAIT_START";
                    buffer = [];
            end
        end

    catch ME
        warning(['Erro no callback de leitura: ' ME.message]);
        state = "WAIT_START";
        buffer = [];
    end
end