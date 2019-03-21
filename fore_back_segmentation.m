clear;
close all;
fclose all;
addpath(genpath('gco'));

img_color = imread('global.jpg');
img_depth = imread('global.tiff');

% superpixel segmentation
[L, N] = superpixels(img_color, 3000, 'Compactness', 5);
BW = boundarymask(L);
figure, imshow(imoverlay(img_color, BW, 'cyan'), ...
    'InitialMagnification', 67);

% find neighbors
neighbors = cell(N, 1);
mean_color = zeros(N, 3);
mean_depth = zeros(N, 1);
cost_vol = zeros(N, 1);
se = strel('rectangle', [3, 3]);
red_ch = img_color(:, :, 1);
green_ch = img_color(:, :, 2);
blue_ch = img_color(:, :, 3);
for spind = 1:N
   mask = L;
   mask(mask ~= spind) = 0;
   mask2 = imdilate(mask, se, 'same');
   mask_res = mask2 - mask;
   neighbor = unique(L(mask_res ~= 0));
   neighbors{spind} = neighbor;
   depth_pixels = img_depth(L == spind);
   mean_depth(spind, 1) = mean(depth_pixels);
   
   red_pixels = red_ch(L == spind);
   mean_color(spind, 1) = mean(red_pixels);
   green_pixels = green_ch(L == spind);
   mean_color(spind, 2) = mean(green_pixels);
   blue_pixels = blue_ch(L == spind);
   mean_color(spind, 3) = mean(blue_pixels);
end

% set foreground background cost volume
for spind = 1:N
   depth_sps = zeros(1 + size(neighbors{spind}, 1), 1);
   sp_inds = [spind; neighbors{spind}];
   for sp_ind2 = 1:size(sp_inds, 1)
       depth_sps(sp_ind2, 1) = mean_depth(sp_inds(sp_ind2, 1), 1);
   end
   % get min and max depth
   min_depth = min(depth_sps);
   max_depth = max(depth_sps);
   mean_depth_spg = mean(depth_sps);
   std_depth_spg = std(depth_sps);
   % if cross edges
   if max_depth - min_depth > 10  %越小mask越多
      for sp_ind2 = 1:size(sp_inds, 1)
          if depth_sps(sp_ind2, 1) < mean_depth_spg - std_depth_spg
              cost_vol(sp_inds(sp_ind2, 1), 1) = cost_vol(sp_inds(sp_ind2, 1), 1) - 1;
          elseif depth_sps(sp_ind2, 1) > mean_depth_spg + std_depth_spg
              cost_vol(sp_inds(sp_ind2, 1), 1) = cost_vol(sp_inds(sp_ind2, 1), 1) + 1;
          end
      end
   end
end

% visualize 1
img2 = img_color;
L3 = cat(3, L, L, L);
for spind = 1:N
   if cost_vol(spind, 1) > 0
       img2(L3 == spind) = 255;
   elseif cost_vol(spind, 1) < 0
       img2(L3 == spind) = 0;
   end
end
figure, imshow(img2 * 0.5 + img_color);

% MRF refine
data_cost = zeros(2, N);
smooth_cost = [0, 1; 1, 0];
neighbor_conn = zeros(N, N);
for spind = 1:N
    data_cost(1, spind) = 10 + cost_vol(spind, 1);
    data_cost(2, spind) = 10 - cost_vol(spind, 1);
    for neind = neighbors{spind}
        cost = max(50 - norm(mean_color(spind, :) - mean_color(neind, :)) / 3, 0);
        neighbor_conn(spind, neind) = cost;
        neighbor_conn(neind, spind) = cost;
    end
end
h = GCO_Create(N, 2);
GCO_SetDataCost(h, data_cost * 500);
GCO_SetSmoothCost(h, smooth_cost);
GCO_SetNeighbors(h, neighbor_conn);
GCO_Expansion(h);
L_mrf = GCO_GetLabeling(h);

% visualize 2
img2 = img_color;
L3 = cat(3, L, L, L);
for spind = 1:N
%    if cost_vol(spind, 1) > 0
%        img2(L3 == spind) = 255;
%    elseif cost_vol(spind, 1) < 0
%        img2(L3 == spind) = 0;
%    end
   if L_mrf(spind, 1) == 2
       img2(L3 == spind) = 255;
   elseif L_mrf(spind, 1) == 1
       img2(L3 == spind) = 0;
   end
end
figure, imshow(img2 * 0.5 + img_color);

msk = imdilate(logical(img2(:,:,1)),ones(9));

imwrite(msk, 'mask.png');