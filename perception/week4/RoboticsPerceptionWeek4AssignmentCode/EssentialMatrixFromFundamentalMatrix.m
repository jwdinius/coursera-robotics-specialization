function E = EssentialMatrixFromFundamentalMatrix(F,K)
%% EssentialMatrixFromFundamentalMatrix
% Use the camera calibration matrix to esimate the Essential matrix
% Inputs:
%     K - size (3 x 3) camera calibration (intrinsics) matrix for both
%     cameras
%     F - size (3 x 3) fundamental matrix from EstimateFundamentalMatrix
% Outputs:
%     E - size (3 x 3) Essential matrix with singular values (1,1,0)

% compute K'FK
E = K'*F*K;

% compute svd of E to properly scale
[U,D,V] = svd(E);

% normalize to get (1,1,0) as singular values
D(1,1) = 1;
D(2,2) = 1;

% remultiply
E = U*D*V';

% scale to have norm 1
E = E / norm(E);

