function [ im ] = rotImg3( img, teta, ax, method, pad )
%rotImg3 rotates 3 image
% --- Syntax
%   [ im ] = rotImg3( img, teta, ax, method )
% img is the 3D image, teta is the angle in radians, ax is the 
% axis of rotation for exmple [1 0 0], method can be nearest for nearest
% neighbor of linear interpolation. pad =(true|false) is wheter or not to pad the object
% with nan or to crop the image after the rotation and leave it same size
%
% example use for rotating a cylinder:
% nS = 30; % cylynder size
% cylBlock = repmat([1 zeros(1,nS-2) 1], nS,1);
% cyl = zeros(nS,nS,nS);
% cyl(:,:,1) = ones(nS,nS);
% cyl(:,:,end) = ones(nS,nS);
% cyl(:,:,2:end-1) = repmat( cylBlock, [1 1 nS-2]);
% rotatedCyl = rotImg3(double(cyl), 1*pi/4 , [0 1 0 ]);
% isosurface(rotatedCyl);

if ~exist('method', 'var')
    method = 'linear';
end
if ~exist('pad', 'var')
    pad = true;
end

if teta == 0
    im = img;
    return;
end

sz = size(img);
ratM = rotationmat3D(teta, ax);

% padding image
if pad
    s = max(sz);
    imagepad = zeros([3 * s, 3 * s, 3 * s]);
    ss = floor((3*s - sz) / 2);
    imagepad(ss(1)+1:ss(1)+sz(1), ...
        ss(2)+1:ss(2)+sz(2), ...
        ss(3)+1:ss(3)+sz(3)) = img;
else
    imagepad = img;
end

[nd1, nd2, nd3] = size(imagepad);
midx=(nd1+1)/2;
midy=(nd2+1)/2;
midz=(nd3+1)/2;

% rotate about center
ii = zeros(size(imagepad));
idx = find( ~ii );
[X, Y, Z] = ind2sub (size(imagepad) , idx ) ;


XYZt = [X(:)-midx Y(:)-midy Z(:)-midz]*ratM;
XYZt = bsxfun(@plus,XYZt,[midx midy midz]);

xout = XYZt(:,1);
yout = XYZt(:,2);
zout = XYZt(:,3);

imagerotF = interp3(imagepad, yout, xout, zout, method);
im = reshape(imagerotF, size(imagepad));

%shrink image to use minimal size
idx=find(abs(im)>0);
[mx, my, mz] = ind2sub(size(im), idx);
im = im(min(mx):max(mx), min(my):max(my), min(mz):max(mz));

end
