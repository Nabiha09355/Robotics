function mask = segmentObject(color_frame)
% Produce a binary mask for red objects using HSV thresholding
% plus morphological noise suppression.

    rgb     = im2double(color_frame);
    hsvImg  = rgb2hsv(rgb);
    h = hsvImg(:,:,1);
    s = hsvImg(:,:,2);
    v = hsvImg(:,:,3);

    % Red hue wraps around 0/1 in HSV
    % Expanded range for red/orange hues, lower saturation/brightness limits
    mask = (h < 0.08 | h > 0.92) & (s > 0.25) & (v > 0.2);

    % Morphological noise suppression
    SE_open  = strel('disk', 3);
    SE_close = strel('disk', 5);
    mask = imopen(mask,  SE_open);
    mask = imclose(mask, SE_close);

    % Fill holes and remove tiny blobs
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 500);
end