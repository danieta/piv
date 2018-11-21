%% Opening files/initialization of matrices
load camera_parameters;
cd um;

images_jpg=dir('*.jpg');
images_mat=dir('*.mat');

images_depth=zeros(480,640,length(images_jpg));

for i=1:length(images_jpg),
    images_rgb(:,:,:,i)=(imread(images_jpg(i).name));
    load(images_mat(i).name);
    images_depth(:,:,i)=double(depth_array)/1000;
    figure(1);
    imagesc(images_depth(:,:,i));
    title("Depth");
end



cd ..;
%% Computing the background for the depth and RGB image sequences given
bg_depth=median(images_depth,3);
figure(2);
imagesc(bg_depth);
title("depth background");


figure(1);clf;

for i=1:length(images_jpg),
    %subtract current image and depth background
    depth_subtracted(:,:,i)=abs(images_depth(:,:,i)-bg_depth)>.20;
    
    %morph filter every depth image. remove noise components
    depth_subtracted(:,:,i)=imopen(depth_subtracted(:,:,i),strel('disk',10));
    
    
    figure(1);
    imagesc(depth_subtracted(:,:,i));
    title("Depth subtracted");
    
    figure(4);
    %[labels, number_of_objects] = bwlabel(depth_subtracted(:,:,i));
    labels = regionprops(depth_subtracted(:,:,i), 'centroid');
    centroids = cat(1, labels.Centroid);
    imshow(depth_subtracted(:,:,i));
    hold on;
    plot(centroids(:,1),centroids(:,2), 'b*')
    hold off;
    title("bwlabel test");
end


% image_1 = images_depth(:,:,1);
% im = double(images_rgb(:,:,:,1));
% 
% 
% Z=double(depth_array(:)')/1000;
% [v u]=ind2sub([480 640],(1:480*640));
% P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
% niu=cam_params.Krgb*[cam_params.R cam_params.T]*[P;ones(1,640*480)];
% u2=round(niu(1,:)./niu(3,:));
% v2=round(niu(2,:)./niu(3,:));
% 
% 
% im2=zeros(640*480,3);
% indsclean=find((u2>=1)&(u2<=641)&(v2>=1)&(v2<=480));
% indscolor=sub2ind([480 640],v2(indsclean),u2(indsclean));
% im1aux=reshape(im,[640*480 3]);
% im2(indsclean,:)=im1aux(indscolor,:);
% 
% pc=pointCloud(P', 'color',uint8(im2));
% figure(1);showPointCloud(pc);