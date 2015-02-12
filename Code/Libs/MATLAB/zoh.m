function Vq = zoh(t,V,tq,keepNaNs)
%ZOH time series interpolation via zero-order hold. Derived and optimized
%from the (undocumented) TSINTERP function in base MATLAB.
%
% Inputs (All should be doubles):
% t: The sample times.  A column vector of monotonically nondecreasing
%   values.
% V: The sample values.  A matrix with the same number of rows as t and as
%   many columns as you have time series.
% tq: The query time values as a column vector.  It must also be
%   monotonically nondecreasing.
% keepNaNs: (Optional)  A logical true or false indicating if NaNs
%   in the sample values V are to be kept as valid observations (true) or
%   if they are to be discarded as invalid before the interpolation (false,
%   the default).  If keepNaNs is false and V has more than 1 column, then
%   any NaNs at a sample time must be NaNs across all the sample values at
%   that time step.  Otherwise, the function must be called on 1 column of
%   V at a time.
%
% Outputs:
% Vq: A matrix with the same number of rows as tq and the same number of
%   columns as V corresponding to the interpolated values of V at the
%   queried time values.
%
% This function is designed primarily for speed and thus has only minimal
% input argument checking.  It may give incorrect answers (rather than an
% error) if given inputs that do not conform to the standards above.

if nargin == 3
    keepNaNs = false;
end

ultraFast = true;
% This flag toggles an "ultra-fast" (but slightly risky) mode.  This is
% important only when there are repeated observations in the original time
% vector t. The "correct" thing to do in this case is to only use the "most
% recent" observation for interpolation-- in other words, use only the row
% of V that corresponds to the "last/bottom" of the repeated entries in t.
%
% The majority of the code below laboriously detects all such duplicate
% times and breaks the problem into however many subproblems it needs.
% That's safe, but potentially slow since it involves multiple function
% calls and growing vectors in a loop.
%
% The faster option is to just call HISTC with duplicate times in t.  This
% possibility is not explicitly discussed in the doc for HISTC: while it
% does allow the binranges input to have repeated values ("...monotonically
% nondecreasing..." instead of "...monotonically increasing..."), it
% doesn't say exactly what the behavior is in that case.  And since HISTC
% is pre-compiled, I can't confirm that it will always do the correct
% thing.  My experience, though, indicates that it does in fact do what we
% want, so I think it's relatively safe to leave the ultraFast flag on.
% Switch it to false if you're worried and don't mind a slight hit on
% performance.

if ~ultraFast
    % Find duplicate times
    diffT = diff(t);
    Iduplicate = [false,(diffT(:)'==0)];
end

if ~ultraFast && any(Iduplicate)
    Tduplicate = unique(t(Iduplicate));
        
    % Interpolate before the first duplicate time
    I = find(Tduplicate(1)==t);
    
    % T(1:I(1)) are all unique and include the first duplicated time
    if tq(1)<Tduplicate(1)
        Vq = zordhold( tq(tq<Tduplicate(1)),t(1:I(1)), V(1:I(1), :), keepNaNs);
    else
        Vq = [];
    end
    
    % Any values of t which overlap the first duplicate interpolant
    % time produce output which is the positionally last value of X at
    % the duplicate interpolant time.  This duplicate value must be
    % repeated for each duplicate value of the interpolate time t.
    Vq = [Vq; repmat(V(I(end), :), [sum(tq==Tduplicate(1)) 1])];
    
    Iklast = I;
    Ik = I;
    for k=2:length(Tduplicate)
        
        % Add interpolated values between Tduplicate(k-1) and
        % Tduplicate(k)
        Ik = find(Tduplicate(k)==t);
        % T(Iklast(end):Ik(1)) are all unique and include the last duplicated
        % time from the previous interval and the first duplicated time
        % from the current interval
        It = tq>Tduplicate(k-1) & tq<Tduplicate(k);
        if any(It)
            Vq = [Vq;zordhold(tq(It),t(Iklast(end):Ik(1)),...
                V(Iklast(end):Ik(1), :), keepNaNs)]; %#ok<AGROW>
        end
        
        % Any values of t which overlap the kth duplicate time produce output
        % which is the positional latest of X at the duplicate times
        
        % Any values of t which overlap the k-th duplicate interpolant
        % time produce output which is the positionally last value of X at
        % the duplicate interpolant time.  This duplicate value must be
        % repeated for each duplicate value of the interpolate time t.
        Vq = [Vq; repmat(V(Ik(end), :), [sum(tq==Tduplicate(k)) 1])]; %#ok<AGROW>
        
        Iklast = Ik;
    end
    
    % Interpolate after duplicate times. T(I(end):end) are all unique
    % and include the last duplicate time,
    if tq(end)>Tduplicate(end)
        Vq = [Vq;zordhold(tq(tq>Tduplicate(end)),t(Ik(end):end),...
            V(Ik(end):end, :), keepNaNs)];
    end

else
    Vq = zordhold(tq,t,V,keepNaNs);
end

function out = zordhold(t,T,X,keepNaNs)

% Initialize output to NaNs.
out = nan(size(t,1), size(X,2));

if keepNaNs
    Tvalid = T;
else
    % Find NaN values.
    nanvals = isnan(X);
    
    if size(X,2) > 1
        % Check if there are any NaNs in X.  If there are, then the entire
        % row must be NaNs, or else we can't filter them out easily.
        if any(any(nanvals, 2) ~= all(nanvals, 2))
            error(['Cannot remove any NaNs from a matrix V unless all the ' ...
                'entries in that row are NaNs.  Please call ZOH again on ' ...
                'one column of V at a time.'])
        else
            nanvals = any(nanvals,2);
        end
    
    end
    
    Tvalid = T(~nanvals);
end

% Use HISTC to allocate the new time vector into bins defined by the
% valid time vector. The second out argument Iref identifies the
% indices of the interpolated time vector in the intervals defined by the
% interpolant time vector.

[~,Iref] = histc(t,Tvalid);

% Iref == 0 is all values in t coming before the first value in Tvalid.
% Leave them as NaNs.
out(Iref>0, :) = X(Iref(Iref>0), :);