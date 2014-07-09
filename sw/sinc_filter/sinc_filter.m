function y = sinc_filter(x,Wn,N,dim)
% Apply a near-ideal low-pass or band-pass brickwall filter
% 
%   y = sinc_filter(x,Wn)
%   y = sinc_filter(x,Wn,N)
%   y = sinc_filter(x,Wn,N,dim)
%   y = sinc_filter(x,Wn,[],dim)
% 
%   y = sinc_filter(x,Wn) applies a near-ideal low-pass or
%   band-pass brickwall filter to the array x, operating
%   along the first non-singleton dimension (e.g. down the
%   columns of a matrix). The cutoff frequency/frequencies
%   are specified in Wn. If Wn is a scalar, then Wn
%   specifies the low-pass cutoff frequency. If Wn is a
%   two-element vector, then Wn specifies the band-pass
%   interval. Wn must be 0.0 < Wn < 1.0, with 1.0
%   corresponding to half the sample rate.
% 
%   The filtering is performed by FFT-based convolution of x
%   with the sinc kernel.
% 
%   y = sinc_filter(x,Wn,N) allows the filter length to be
%   specified. The default value is N=1025. The filter
%   length is doubled in the band-pass case. In either case,
%   if N is even, the final filter length will be N+1.
%   
%   y = sinc_filter(x,Wn,N,dim) applies the specified filter
%   along the dimension dim.
%   
%   y = sinc_filter(x,Wn,[],dim) applies the specified
%   filter along the dimension dim using the default filter
%   length.
% 
%   See also CONV_FFT.

% !---
% ==========================================================
% Last changed:     $Date: 2013-12-20 11:44:30 +0000 (Fri, 20 Dec 2013) $
% Last committed:   $Revision: 265 $
% Last changed by:  $Author: ch0022 $
% ==========================================================
% !---

%% test input

assert(nargin>=2,'Not enough input arguments')
assert((numel(Wn)==1 || numel(Wn)==2) & isnumeric(Wn),'Wn must be a scalar or two-element vector.')
assert(isnumeric(x),'x must be a numeric array.')
assert(all(Wn<=1) & all(Wn>=0),'Wn must be 0.0 < Wn < 1.0, with 1.0 corresponding to half the sample rate.')
dims = size(x);
if nargin<4
    dim = [];
else
    assert(isint(dim) & numel(dim)==1,'dim must be an integer')
    assert(dim<=length(dims),'dim must be less than or equal to the number of dimensions in x')
end
if nargin<3
    N = [];
elseif ~isempty(N)
    assert(isscalar(N) & isint(N),'N must be an integer scalar.')
end

%% assign defaults

if isempty(N)
    N = 1025;
end
if isempty(dim)
    dim = find(dims>1,1,'first');
end

%% reshape input to matrix with requested dim made as first dimension

% reshape data so function works down columns
order = mod(dim-1:dim+length(dims)-2,length(dims))+1;
x = permute(x,order);
dims_shift = dims(order);
x = reshape(x,dims_shift(1),numel(x)/dims_shift(1));
y = zeros(size(x));

%% create filter kernel

if numel(Wn)==1 % low-pass
    n = -floor(N/2):floor(N/2); % create time base
    B = sinc_kernel(Wn,n); % make kernel
else % band-pass
    n = -N:N; % create time base
    B = sinc_kernel(Wn(2),n)-sinc_kernel(Wn(1),n); % make kernel
end

%% apply filter

for N = 1:size(y,2)
    y(:,N) = conv_fft(x(:,N),B,'same');
end

%% reshape out to match input

y = reshape(y,dims_shift);
y = ipermute(y,order);

% end of sinc_filter()

% ----------------------------------------------------------
% Local functions:
% ----------------------------------------------------------

% ----------------------------------------------------------
% sinc_kernel: Make sinc kernel
% ----------------------------------------------------------
function k = sinc_kernel(Wn,n)

k = sinc(Wn*n).*Wn;

% ----------------------------------------------------------
% isint: Test whether x is integer value (not type)
% ----------------------------------------------------------
function y = isint(x)

y = x==round(x);

% [EOF]


