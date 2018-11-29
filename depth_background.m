%% Opening files/initialization of matrices
load camera_parameters;
cd um;

images_jpg=dir('*.jpg');
images_mat=dir('*.mat');

images_depth=zeros(480,640,length(images_jpg));

for frame_number=1:length(images_jpg)
    images_rgb(:,:,:,frame_number)=(imread(images_jpg(frame_number).name));
    load(images_mat(frame_number).name);
    images_depth(:,:,frame_number)=double(depth_array)/1000;
end

number_of_frames = frame_number;

cd ..;
%% Computing the background for the depth sequences given
bg_depth=median(images_depth,3);
figure(2);
imagesc(bg_depth);
title("depth background");

filter_criteria = strel('disk',20);

for frame_number=1:length(images_jpg),
    %subtract current image and depth background
    foreground_depth(:,:,frame_number)=abs(images_depth(:,:,frame_number)-bg_depth)>.50;
    
    %morph filter every depth image. remove noise components
    foreground_depth_morphed(:,:,frame_number)=imopen(foreground_depth(:,:,frame_number),filter_criteria);
    
    
    figure(1);
    imagesc(foreground_depth_morphed(:,:,frame_number));
    title("Foreground morphed");
    
%     figure(4);
%     [labels, number_of_objects] = bwlabel(foreground_depth(:,:,frame_number));
%     labels = regionprops(foreground_depth_morphed(:,:,frame_number));
%     centroids = cat(1, labels.BoundingBox);
%     imshow(foreground_depth_morphed(:,:,frame_number));
%     hold on;
%     plot(centroids(:,1),centroids(:,2), 'b*')
%     hold off;
%     title("bwlabel test");
end


%% Removing noise components that the morph filter didn't

%which image to generate pointcloud of
image_nr = 5;

%Get the moving elements in the image in question
[labels, number_of_objects] = bwlabel(foreground_depth_morphed(:,:,image_nr));
bounding_boxes = regionprops(labels, 'BoundingBox');

figure(10);
imagesc(foreground_depth_morphed(:,:,image_nr));

%Remove objects that occupy less pixels than a given threshold
for i=1:number_of_objects
    %dimensions of the current box
    start_x = int32(bounding_boxes(i).BoundingBox(1));
    start_y = int32(bounding_boxes(i).BoundingBox(2));
    width = int32(bounding_boxes(i).BoundingBox(3));
    height = int32(bounding_boxes(i).BoundingBox(4));
    if (width * height <= 800)
        for j=start_x:start_x+width
            for k=start_y:start_y+height
                foreground_depth_morphed(k,j,image_nr) = 0;
            end
        end
    end
end

figure(11);
imagesc(foreground_depth_morphed(:,:,image_nr));
% hold on;

%Get the moving elements that remain after removing noise
[labels, number_of_objects] = bwlabel(foreground_depth_morphed(:,:,image_nr));
bounding_boxes = regionprops(labels, 'BoundingBox');
for i=1:number_of_objects
    rectangle('Position', bounding_boxes(i).BoundingBox, 'EdgeColor','r', 'LineWidth', 3);
    hold on;
end

%% Testing the pointcloud generation
object_label=reshape(double(labels),[],1);
depth_reshaped=reshape(double(images_depth(:,:,image_nr)),[],1);

objects = struct;

for i=1:number_of_objects
    objects(i).indeces = find(labels == i);
    objects(i).zvalues = depth_reshaped(objects(i).indeces(:));
    %by manipulating to array Pwe dont need this in the struct
%     objects(i).xvalues =   
%     objects(i).yvalues = 
%     objects(i).zmax = max(objects(i).zvalues(:));
%     A = objects(i).zvalues(:);
%     objects(i).zmin = min( A (A > 0) );
%     objects(i).x
end

% Keep the pixels that are non-zero, that is, the pixels of the moving objects
for i=1:number_of_objects
    index_objects(:,1)= find(foreground_depth_morphed(:,:,image_nr) ~= 0);
    index_objects(:,2) = object_label(index_objects(:,1));
    index_objects(:,3) = depth_reshaped(index_objects(:,1));
end


%corresponding depth foreground and rgb of this image
imd = images_depth(:,:,image_nr);
im = double(images_rgb(:,:,:,image_nr));

%reshaping the foreground depth image so it is a long vector instead of
% being a matrix. this is just so that we can do fast computations in 
% matlab
Z=double((reshape(double(imd),[],1))');

%dont really know whats happening here
[v u]=ind2sub([480 640],(1:480*640));
P=inv(cam_params.Kdepth)*[Z.*u; Z.*v; Z];


%deletes the background from the PC
% a=1;
% for frame_number=1:length(P)
%     if frame_number==index_objects(a,1)
%         if a < length(index_objects)
%             a=a+1;
%         end
%     else 
%         P(:, frame_number)=[0 0 0];
%     end
% end

%computing the homogeneous coordinates of the image projected from the
%pointcloud
niu=cam_params.Krgb*[cam_params.R cam_params.T]*[P;ones(1,640*480)];
u2=round(niu(1,:)./niu(3,:));
v2=round(niu(2,:)./niu(3,:));

%dont really know whats going on here either
im2=zeros(640*480,3);
indsclean=find((u2>=1)&(u2<=641)&(v2>=1)&(v2<=480));
indscolor=sub2ind([480 640],v2(indsclean),u2(indsclean));

%error we were passing im = double(images_rgb(:,:,:,image_nr));
%so the image was without depth 
%corrected with im_pC
im1aux=reshape(im,[640*480 3]);
im2(indsclean,:)=im1aux(indscolor,:);
% project in the point cloud only the objectes detected

% for i=1:number_of_objects
    PCdiga=P(:,objects(1).indeces(:));
    pc=pointCloud(PCdiga', 'color',uint8(im2(objects(1).indeces(:),:)));
    figure(3);showPointCloud(pc);
    hold on;
    % 

    B = P(1,objects(1).indeces(:));
    x_max = max( B( B < 0 ) );
    x_min = min( B );
    C = P(2,objects(1).indeces(:));
    y_max = max( C );
    y_min = min( C );
    D = P(3,objects(1).indeces(:));
    z_max = max( C );
    z_min = min( C (C < 0));
    figure(25)
    plot( plot::Box([x_min, y_min, z_min], [x_max, y_max, z_max], Filled = FALSE, LineColor = RGB::Red),
         Axes = None, Scaling = Constrained);
      
%       b := plot::Box([x_min, y_min, z_min], [x_max, y_max, z_max]);
%       plot(b)
         
% end


% depth_1 = foreground_depth(:,:,5);
% figure(28); imagesc(depth_1);
% depth_1=double((reshape((depth_1),[],1)));
% 
% b = foreground_depth_morphed(:,:,5);
% c = foreground_depth(:,:,5);
% d = images_depth(:,:,5).*b;
% e = reshape(d, [], 1);
% f = logical(images_rgb(:,:,:,5)).*b;
% %f = images_rgb(:,:,:,5);
% 
% xyz1=get_xyz_asus(e',[480 640],(1:640*480)', cam_params.Kdepth,1,0);
% rgbd1 = get_rgbd(xyz1, f, cam_params.R, cam_params.T, cam_params.Krgb);
% 
% pc2=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
% figure(27);showPointCloud(pc2);