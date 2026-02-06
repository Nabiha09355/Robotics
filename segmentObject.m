function mask = segmentObject(color_frame)
    rgb = im2double(color_frame);
    hsvImg = rgb2hsv(rgb);

    h = hsvImg(:,:,1);
    s = hsvImg(:,:,2);
    v = hsvImg(:,:,3);

    mask = (h < 0.05 | h > 0.95) & (s > 0.4) & (v > 0.3);
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 500);
end
