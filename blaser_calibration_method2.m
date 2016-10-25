camera = 'red';
v = VideoReader([camera,'.avi']);
squareSize = 3.8; % mm
boardSize = [7,10];
[worldPoints] = generateCheckerboardPoints(boardSize,squareSize);
n_frame = round(v.Duration*v.FrameRate/7)-2;
threshold = 100;
imagePoints_all = [];
laserPoints_all = [];
frame = [];
% obj = VideoWriter('blaser_calib.avi');
% open(obj)
for i = 1:n_frame
    I = read(v,i*7);
%     I = read(v,i);
    try
        [imagePoints,boardSize_detected] = detectCheckerboardPoints(I);
    catch
        continue
    end
    if norm(boardSize - boardSize_detected)>0.1
        continue
    end
    try
        imagePoints_all = cat(3,imagePoints_all,flip(imagePoints,1));
    catch
        continue
    end 
    imshow(I);
    pixel_data_valid = extractPixelDataFromImg(I, threshold);
    coeffs = polyfit(pixel_data_valid(:,1), pixel_data_valid(:,2), 1);
    [B,fitness] = sort(abs(polyval(coeffs, pixel_data_valid(:,1))-pixel_data_valid(:,2)));
    laserPoints_all = cat(3,laserPoints_all,pixel_data_valid(fitness(1:100),:));
    
    hold on; 
    scatter(pixel_data_valid(fitness(1:1000),1),pixel_data_valid(fitness(1:1000),2),30,'g','fill');
%     scatter(pixel_data_valid(:,1),pixel_data_valid(:,2));
    plot(imagePoints(:,1), imagePoints(:,2), 'r');
    scatter(imagePoints(1,1), imagePoints(1,2), 'p');
    frame = [frame,i*7];
%     if i == 1
%         pause();
%     end
    drawnow;
%     imwrite(video,[num2str(i),'.tif']);
%     frame = getframe(gcf);
%     writeVideo(obj,frame);

end
% close(obj);
% InitialIntrinsicMatrix = [20000,0,0;0,20000,0;640,360,1];
% [cameraParams,imagesUsed,estimationErrors] = estimateCameraParameters(imagePoints_all,worldPoints,'InitialIntrinsicMatrix',InitialIntrinsicMatrix);
[cameraParams,imagesUsed,estimationErrors] = estimateCameraParameters(imagePoints_all,worldPoints);

%%
K = cameraParams.IntrinsicMatrix';
laser_point = [];
for i = 1:size(frame,2)
    R = cameraParams.RotationMatrices(:,3,i);
    t = cameraParams.TranslationVectors(i,:);
    d = -t*R;
    point_3d = K\[laserPoints_all(:,:,i)';ones(1,size(laserPoints_all,1))];
    z = -d./(point_3d'*R);
    laser_point = [laser_point,point_3d.*[z';z';z']];
end
pts = [laser_point', ones(size(laser_point,2),1)];

% from K = 3*3 to K = 4*4
K_expand = [K,zeros(3,1);zeros(1,3),1];
A = pts * (K_expand ^ (-1))';
[U,S,V] = svd(A);

gc = V(:,4)';

% gc is the camera parameters needed from distance detection
% it can be verified that gc(1)^2+gc(2)^2+gc(3)^2 = 1
gc_square_sum = sqrt(gc(1)^2+gc(2)^2+gc(3)^2);
gc_gain = 1/gc_square_sum;
gc = V(:,4) * gc_gain;
gc = gc'

figure
scatter3(laser_point(1,:),laser_point(2,:),laser_point(3,:));

fileID = fopen([camera,'.yaml'],'w');
fprintf(fileID,'Camera intrisic matrix:\n');
fprintf(fileID,'%f %f %f\n',cameraParams.IntrinsicMatrix);
fprintf(fileID,'laser plane in camera frame:\n');
fprintf(fileID,'%f %f %f %f\n',gc);
fclose(fileID);
