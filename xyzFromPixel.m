function cam_coord = xyzFromPixel(x,y)
% function will return a column vector = [X;Y;Z] if given pixel coord (x,y)
% Z is the distance
% x is the col pixel index of image!!!!!
% y is the row pixel index of image!!!!!

K = [1269.706127 0.000000 627.072628;
0.000000 1279.888426 394.829375;
0.000000 0.000000 1.000000];

adjust = 1.00;
gc = [-0.071478 0.962445 0.261897 -0.021414];
K_expand_inv =  [inv(K), zeros(3,1);
                 zeros(1,3),1];

Ad = gc*K_expand_inv;

dist = -Ad(4)/(Ad(1)*x + Ad(2)*y + Ad(3))*adjust;

pixel_coord = [x;y;1];

proj_coord = pixel_coord * dist;

cam_coord = K \ proj_coord;
