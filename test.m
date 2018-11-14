load camera_parameters.mat
cd um;
loaded_images_rgb=dir('*.jpg');
loaded_images_depth=dir('*.mat');

images_rgb=zeros(480,640,length(loaded_images_rgb));
images_depth=zeros(480,640,length(loaded_images_rgb));
for i=1:length(loaded_images_rgb),
    images_rgb(:,:,i)=rgb2gray(imread(loaded_images_rgb(i).name));
    load(loaded_images_depth(i).name);
    images_depth(:,:,i)=double(depth_array)/1000;
    figure(1)
    imshow(uint8(images_rgb(:,:,i)));
    figure(2);
    imagesc(images_depth(:,:,i));
    %colormap(gray);
    im = images_rgb(:,:,i);
    
    Kd=Depth_cam.K;
    Z=double(depth_array(:)')/1000;
    % Compute correspondence between two imagens in 5 lines of code
    [v u]=ind2sub([480 640],(1:480*640));
    P=inv(Kd)*[Z.*u ;Z.*v;Z];
    niu=RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*[P;ones(1,640*480)];
    u2=round(niu(1,:)./niu(3,:));
    v2=round(niu(2,:)./niu(3,:));
    % Compute new image - the easy understandable way
    im3=zeros(size(im));
    for i=1:length(v2),
        if ((v2(i)>0 &v2(i)<482)&&(u2(i)>0 & u2(i)<642)),
            vv=max([1 min([v2(i) 480])]);
            uu=max([1 min([u2(i) 640])]);
            im3(v(i),u(i),:)=im(vv,uu,:);
        end
    end
    im2=zeros(640*480,3);
    pc=pointCloud(P', 'color',uint8(im2));
    figure(3);showPointCloud(pc);
    % figure(4);imshow(uint8(reshape(im2,[480,640,3])));
end
%%
bgdepth=median(images_depth,3);
bggray=median(images_rgb,3);
figure(1);
subplot(211);imagesc(bgdepth);
subplot(212);imagesc(bggray);
%%
% Bg subtraction for depth (try with gray too)
figure(1);clf;
figure(2);clf;
for i=1:length(loaded_images_rgb),
    imdiff=abs(images_depth(:,:,i)-bgdepth)>.20;
    imgdiffiltered=imopen(imdiff,strel('disk',5));
    figure(1);
    imagesc([imdiff imgdiffiltered]);
    title('Difference image and morph filtered');
    colormap(gray);
    figure(2);
    imagesc([images_depth(:,:,i) bgdepth]);
    title('Depth image i and background image');
    figure(3);
    imagesc(bwlabel(imgdiffiltered));
    title('Connected components');
end