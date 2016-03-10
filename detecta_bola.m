% Detect gripper and object through circular Hough transform and color
% subtraction

function [ Pball,Area,h,param ] = detecta_bola( param, varargin )

    tic

    Pball = 'Não detectado';
    Area = 'Não detectado';

    % Coordenadas para a janela
    Lxneg = param(1);
    Lxpos = param(2);
    Lyneg = param(3);
    Lypos = param(4);
    xc = param(5);
    yc = param(6);

    % Captura tela RGB
    [~, RGB]=Kinect();
    data = RGB/255;

    % Processamento de cor da imagem capturada
    color = 1; % R=1, G=2, B=3
    diff_im = imsubtract(data(max(yc-Lyneg,1):min(yc+Lypos,480),max(xc-Lxneg,1):min(xc+Lxpos,640),color),...
        rgb2gray(data(max(yc-Lyneg,1):min(yc+Lypos,480),max(xc-Lxneg,1):min(xc+Lxpos,640),:)));
    diff_im = medfilt2(diff_im, [4 4]);
    if color==1
        diff_im = im2bw(diff_im,0.10 ); % red
    elseif color==2
        diff_im = im2bw(diff_im,0.03); % green
    else
        diff_im = im2bw(diff_im,0.03); % blue
    end
    diff_im = bwareaopen(diff_im,30);
%     bw = bwlabel(diff_im, 8);
    stats = regionprops(diff_im, 'BoundingBox', 'Centroid','Area');

    % Show images
    if (~isempty(varargin))&&(strcmp(varargin{1},'on'))
        imshow(data);
        refresh();
        hold on;
    end

    if ~isempty(stats)      %-----> se houver algum objeto sendo detectado

        % Limpa variáveis anteriores
        clear real_centers;
        clear area;
        clear Pdtil;
        clear pzb;
        clear ptarget;
        clear Pball;
        clear z_o;
        geomcenter = [0 0];

        % Cria célula para armazenar as submatrizes de profundidade
        % necessárias
%         ret_depth=cell(length(stats),1);

        % Análise de propriedades dos objetos detectados
        for i=1:length(stats)

            % Plota retângulos em volta dos obj. detectados
            real_centers(:,:,i) = [xc-Lxneg yc-Lyneg] + stats(i).Centroid;
            % subplot(2,1,1);
%             rectangle('Position',stats(i).BoundingBox + [xc-Lxneg yc-Lyneg 0 0],'EdgeColor','r','LineWidth',1);
            %subplot(2,1,2);
            %rectangle('Position',stats(i).BoundingBox,'EdgeColor','r','LineWidth',1);
            area(i) = stats(i).Area;

            % Calcula centro geométrico dos objetos vermelhos e atualiza o
            % tamanho do espaço da janela para que todos os objetos
            % caibam
            geomcenter = geomcenter + real_centers(:,:,i);
            if (round(stats(i).BoundingBox(1))<=1)
                Lxneg=Lxneg+1;
            elseif (round(stats(i).BoundingBox(1))+round(stats(i).BoundingBox(3))>=(Lxpos+Lxneg))
                Lxpos=Lxpos+1;
            elseif (round(stats(i).BoundingBox(2))<=1)
                Lyneg=Lyneg+1;
            elseif (round(stats(i).BoundingBox(2))+round(stats(i).BoundingBox(4))>=(Lypos+Lyneg))
                Lypos=Lypos+1;
                % Limites da janela
                %             elseif Lxpos>50
                %                 Lxpos=Lxpos-1;
                %             elseif Lxneg>50
                %                 Lxneg=Lxneg-1;
                %             elseif Lypos>50
                %                 Lypos=Lypos-1;
                %             elseif Lyneg>50
                %                 Lyneg=Lyneg-1;
            end
        end

        [~,max_area_index] = max(area); % -----> seleciona objeto com maior área (bola)

        rectangle('Position',stats(max_area_index).BoundingBox + [xc-Lxneg yc-Lyneg 0 0],'EdgeColor','r','LineWidth',1);
        
        for i=1:length(stats)
            Pdtil(:,:,i) = [real_centers(:,1,i) real_centers(:,2,i) 1]';
            % ptarget(:,:,i) = pztarget(i)*inv_RGB_intrinsics*Pdtil(:,:,i); ----> conversão para o espaço op. no frame da camera (dependente da calibração)
        end

        % Imprime na tela as coordenadas do centróide e área da bola
        if ~isempty(stats)
            Pball = Pdtil(:,:,max_area_index); % ----> centróide da bola
%             fprintf(1,'P = [%f %f %f]^T \n\n',Pball(1),Pball(2),Pball(3));
            Area = area(max_area_index); % ----> profundidade da bola
%             fprintf(1,'Area = %f [mm^2] \n\n',Area);
%             if strcmp(varargin{length(varargin)},'depth')
%                 Depth = median(median(depth(round(yc)-Lxneg:round(yc)+Lxpos,round(xc)-Lyneg:round(xc)+Lypos)))+15;
%             else Depth = 1;
%             end
%             fprintf(1,'Depth = %f [mm] \n\n',Depth);
            % Plot de linhas verticais/horizontais na imagem
%             line([Pball(1) Pball(1)],[0 480],'Color','r');
%             line([640 0],[Pball(2) Pball(2)],'Color','r')
        else
            Pball = 'Não detectado';
            %     fprintf(1,'%s \n\n',Pball);
            Area = 'Não detectado';
            %     fprintf(1,'%s \n\n',Area);
%             Depth = 'Não detectado';
        end

        % Previne que a janela torne-se pequena demais e suma
        geomcenter = geomcenter/length(stats);
        if (geomcenter(1)<=320)
            xc = max(1+Lxneg,geomcenter(1));
        else xc = min(640-Lxpos,geomcenter(1));
        end
        if geomcenter(2)<=240
            yc = max(1+Lyneg,geomcenter(2));
        else yc = min(480-Lypos,geomcenter(2));
        end
    
        % Atualiza array de parametros
        param = [Lxneg,Lxpos,Lyneg,Lypos,xc,yc];

    end
    
     % Atualiza gráficos, verifica a frequência de operação do programa
    if (~isempty(varargin))&&(strcmp(varargin{1},'on'))
        hold off
        pause(0.001);
    end
    
    h = toc;

end