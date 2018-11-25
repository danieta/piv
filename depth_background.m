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

filter_criteria = strel('disk',10);

for i=1:length(images_jpg),
    %subtract current image and depth background
    foreground_depth(:,:,i)=abs(images_depth(:,:,i)-bg_depth)>.50;
    
    %morph filter every depth image. remove noise components
    foreground_depth_morphed(:,:,i)=imopen(foreground_depth(:,:,i),filter_criteria);
    
    
    figure(1);
    imagesc(foreground_depth_morphed(:,:,i));
    title("Foreground morphed");
    
%     figure(4);
%     %[labels, number_of_objects] = bwlabel(depth_subtracted(:,:,i));
%     labels = regionprops(foreground_depth_morphed(:,:,i), 'centroid');
%     centroids = cat(1, labels.Centroid);
%     imshow(foreground_depth_morphed(:,:,i));
%     hold on;
%     plot(centroids(:,1),centroids(:,2), 'b*')
%     hold off;
%     title("bwlabel test");
%     
%     figure(5);
%     imshow(imdilate(foreground_depth_morphed(:,:,i), filter_criteria));
%     title("dilated");
end

%% Testing the pointcloud generation

%which image to generate pointcloud of
image_nr = 5;
% figure(4);
% imagesc(foreground_depth_morphed(:,:,image_nr));
% title("Foreground morphed same as pC");
% figure(5);
% imagesc(images_depth(:,:,image_nr));
% title("Depth same as pC");
%finding to blob in the filtered image
index_filtered_to_pc = find(foreground_depth_morphed(:,:,image_nr)== 1);

%corresponding depth foreground and rgb of this image
imd = images_depth(:,:,image_nr);
im = double(images_rgb(:,:,:,image_nr));

%reshaping the foreground depth image so it is a long vector instead of
% being a matrix. this is just so that we can do fast computations in 
% matlab

foreground_reshaped=reshape(double(imd),[],1);
Z=double(foreground_reshaped');

%dont really know whats happening here
[v u]=ind2sub([480 640],(1:480*640));
P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];


%deletes the background from the PC
a=1;
for i=1:length(P)
    if i==index_filtered_to_pc(a)
        if a < length(index_filtered_to_pc)
            a=a+1;
        end
    else 
        P(:, i)=[0 0 0];
    end
end

niu=cam_params.Krgb*[cam_params.R cam_params.T]*[P;ones(1,640*480)];
u2=round(niu(1,:)./niu(3,:));
v2=round(niu(2,:)./niu(3,:));

%or here
im2=zeros(640*480,3);
indsclean=find((u2>=1)&(u2<=641)&(v2>=1)&(v2<=480));
indscolor=sub2ind([480 640],v2(indsclean),u2(indsclean));

%error we were passing im = double(images_rgb(:,:,:,image_nr));
%so the image was without depth 
%corrected with im_pC
im1aux=reshape(im,[640*480 3]);
im2(indsclean,:)=im1aux(indscolor,:);

pc=pointCloud(P', 'color',uint8(im2));
figure(3);showPointCloud(pc);