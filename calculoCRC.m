function [ msg ] = calculoCRC( message )
%Função para o cálculo do CRC de cada mensagem enviada.
%Recebe um número decimal como argumento
N = length(message);
crc = hex2dec('ffff');
polynomial = hex2dec('a001');
for i = 1:N
    crc = bitxor(crc,message(i)); % problema aqui também... não sei o que é
    for j = 1:8
        if bitand(crc,1)
            crc = bitshift(crc,-1);
            crc = bitxor(crc,polynomial);
        else
            crc = bitshift(crc,-1);
        end
    end
end
lowByte = bitand(crc,hex2dec('ff'));
highByte = bitshift(bitand(crc,hex2dec('ff00')),-8);
amsg = message; % problema resolvido aqui?
amsg(N+1) = lowByte; % problema aqui. acho que o simulink não permite modificar o tamanho das arrays dinamicamente
amsg(N+2) = highByte;
msg = amsg;
end

