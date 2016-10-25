function coord_data_valid = reconstructCamCoordFromPixelCoord(pixel_data_valid)
% input the array of pixel coord of points [x1,y1; x2,y2;...], output the
% cam coord of points[X1,Y1,Z1; X2,Y2,Z2;...], by calling function
% xyzFromPixel()

coord_data_valid = zeros(size(pixel_data_valid,1),3);

% estimate the distance and get (X,Y,Z)
for i = 1:size(pixel_data_valid,1)
    cam_coord = xyzFromPixel(pixel_data_valid(i,1),pixel_data_valid(i,2));
    coord_data_valid(i,:) = cam_coord';
end