%% Opening files/initialization of matrices
cd um;
images_jpg=dir('*.jpg');
images_mat=dir('*.mat');

images_gray=zeros(480,640,3,length(images_jpg));
images_depth=zeros(480,640,length(images_jpg));

for i=1:length(images_jpg),
    images_gray(:,:,:,i)=(imread(images_jpg(i).name));
    load(images_mat(i).name);
    images_depth(:,:,i)=double(depth_array)/1000;
    figure(1)
    imshow(uint8(images_gray(:,:,:,i)));
    title("Gray");
    figure(2);
    imagesc(images_depth(:,:,i));
    title("Depth");
    %pause(.2);
end


%% Computing the background for the depth and RGB image sequences given
background_depth=median(images_depth,3);
figure(5);
imagesc(background_depth);
title("depth background");
background_gray=median(images_gray,4);
figure(6);
imagesc(uint8(background_gray));
title("gray background")
figure(1);
subplot(211);imagesc(background_depth);
subplot(212);imshow(uint8(background_gray));

%xyz1=get_xyz_asus(background_depth(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
%rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);

%% Performing background subtraction for depth (we should try with gray too)
figure(1);clf;
figure(2);clf;

for i=1:length(images_jpg),
    % subtracting the background from the current image to get the
    % difference
    depth_subtracted=abs(images_depth(:,:,i)-background_depth)>.20;
    
    % creating a morphologically filtered version of each subtracted image
    % to reduce noise. 'strel' creates a disk. we filter inside that disk
    image_morphed=imopen(depth_subtracted,strel('disk',5));
    figure(1);
    imagesc([depth_subtracted image_morphed]);
    title('Difference image and morph filtered');
    colormap(gray);
    
    
    figure(2);
    imagesc([images_depth(:,:,i) background_depth]);
    title('Depth image i and background image');
    
    
    figure(3);
    imagesc(bwlabel(image_morphed));
    title('Connected components');
    
    
    %pause(.2);
end
