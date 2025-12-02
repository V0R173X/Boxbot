function frame = build_frame(cmd, id, value)
    % 1. Converte os caracteres para seus valores uint8 (ASCII)
    cmd_byte = uint8(cmd);
    id_byte = uint8(id);
    
    % 2. Converte o valor (uint16) em dois bytes (uint8)
    % typecast é a forma mais fácil de fazer a 'union' do C
    % Ele já trata a ordem little-endian (byte baixo primeiro),
    % exatamente como seu parser espera.
    val_bytes = typecast(uint16(value), 'uint8'); % [val_low, val_high]
    
    % 3. Calcula o Checksum
    % Soma tudo e pega o resto da divisão por 256 (igual ao & 0xFF)
    chk_sum = mod(double(cmd_byte) + double(id_byte) + double(val_bytes(1)) + double(val_bytes(2)), 256);
    chk_byte = uint8(chk_sum);
    
    % 4. Monta o frame de 7 bytes
    frame = [uint8(170), ...  % SOF (0xAA)
             cmd_byte, ...
             id_byte, ...
             val_bytes(1), ... % VAL_L
             val_bytes(2), ... % VAL_H
             chk_byte, ...
             uint8(187)];      % EOF (0xBB)
end