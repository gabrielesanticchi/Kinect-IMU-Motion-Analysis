function [SkeletonConnectionMap] = buildSkeletonConnectionMap()
% Create skeleton connection map to link the joints.

% These are the order of joints returned by the kinect adaptor.
%    SpineBase = 1;
%    SpineMid = 2;
%    Neck = 3;
%    Head = 4;
%    ShoulderLeft = 5;
%    ElbowLeft = 6;
%    WristLeft = 7;
%    HandLeft = 8;
%    ShoulderRight = 9;
%    ElbowRight = 10;
%    WristRight = 11;
%    HandRight = 12;
%    HipLeft = 13;
%    KneeLeft = 14;
%    AnkleLeft = 15;
%    FootLeft = 16;
%    HipRight = 17;
%    KneeRight = 18;
%    AnkleRight = 19;
%    FootRight = 20;
%    SpineShoulder = 21;
%    HandTipLeft = 22;
%    ThumbLeft = 23;
%    HandTipRight = 24;
%    ThumbRight = 25;


SkeletonConnectionMap = [ [4 3];  % Neck
                          [3 21]; % Head
                          [21 2]; % Right Leg
                          [2 1];
                          [21 9];
                          [9 10];  % Hip
                          [10 11];
                          [11 12]; % Left Leg
                          [12 24];
                          [12 25];
                          [21 5];  % Spine
                          [5 6];
                          [6 7];   % Left Hand
                          [7 8];
                          [8 22];
                          [8 23];
                          [1 17];
                          [17 18];
                          [18 19];  % Right Hand
                          [19 20];
                          [1 13];
                          [13 14];
                          [14 15];
                          [15 16];
                        ];
end 