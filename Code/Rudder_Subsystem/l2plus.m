% Set some constants for use with the L2+ trajectory controller
TStar = single(4.5);
IPStar = 0;
InitialPoint = 0;
Turn2Track = 0;
MaxDwnPthStar = 1;
tanIntercept = tan( 45*pi/180 );
switchDistance = 4; % Distance before reaching a waypoint that you will then switch over to the next waypoint (m)