function [ Depth,h ] = detect_depth( param )
tic
    % Coordenadas para a janela
    Lxneg = param(1);
    Lxpos = param(2);
    Lyneg = param(3);
    Lypos = param(4);
    xc = param(5);
    yc = param(6);

    % Captura tela RGB
    [depth, ~] = Kinect();
    
    Depth = median(median(depth(round(yc)-Lxneg:round(yc)+Lxpos,round(xc)-Lyneg:round(xc)+Lypos)))+15;
    
    h = toc;
    
end

