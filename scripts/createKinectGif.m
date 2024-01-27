function createKinectGif(filename, colorImg, metadata, SkeletonConnectionMap)
    % createKinectGif
    % Creates a GIF from Kinect frames
    % filename: Name of the output GIF file
    % colorImg: Color images data from Kinect
    % metadata: Metadata containing joint information
    % SkeletonConnectionMap: Map defining connections between joints

    for Frame_i = 1:5:length(metadata)
        lastframeMetadata = metadata(Frame_i);
        anyBodiesTracked = any(lastframeMetadata.IsBodyTracked ~= 0);
        trackedBodies = find(lastframeMetadata.IsBodyTracked);

        nBodies = length(trackedBodies);
        colorJointIndices = lastframeMetadata.ColorJointIndices(:, :, trackedBodies);
        lastColorImage = colorImg(:, :, :, Frame_i);
        colors = ['r';'g';'b';'c';'y';'m'];

        h = figure();
        imshow(lastColorImage);
        frame = getframe(h); 
        im = frame2im(frame); 
        [imind, cm] = rgb2ind(im, 256); 

        for i = 1:24
            for body = 1:nBodies
                X1 = [colorJointIndices(SkeletonConnectionMap(i,1),1,body) colorJointIndices(SkeletonConnectionMap(i,2),1,body)];
                Y1 = [colorJointIndices(SkeletonConnectionMap(i,1),2,body) colorJointIndices(SkeletonConnectionMap(i,2),2,body)];
                line(X1, Y1, 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '+', 'Color', colors(body));
            end
            hold on;
        end

        if Frame_i == 1 
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf); 
        else 
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append'); 
        end 
        hold off;
        close(h);
    end
end
