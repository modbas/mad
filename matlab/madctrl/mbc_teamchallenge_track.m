function [ track, width ] = mbc_teamchallenge_track(trackNr)
%%
%% Mini-Auto-Drive
%%
%% Team Challenge Tracks
%%
%% Copyright (C) 2017-2020, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%


%% Road Surface
a1total = 2.7; % total surface width [ m ]
a2total = 1.8; % total surface height [ m ]
a1boundary = 0.05; % margin [ m ]
a2boundary = 0.05; % margin [ m ]
a1 = a1total - 2 * a1boundary; % total surface width [ m ]
a2 = a2total - 2 * a2boundary; % total surface height [ m ]

switch trackNr
     case 1
        %% Create clockwise oval track
        width = 300e-3; % track width
        clothoidA = 8; % clothoid parameter
        % determine corner size
        track = mbc_track_create(0, 0, pi/2); % dummy track
        track = create_corner_curve90(track, width, clothoidA, -1); % dummy corner
        cornerSize = track.points{end}.s1;
        % real track
        track = mbc_track_create(a1boundary + 0.5 * width, 0.5 * a2total, pi/2);
        % first vertical segment
        track = mbc_straight_create(track, 0.5 * a2 - cornerSize - 0.5 * width, width);
        % corner
        track = create_corner_curve90(track, width, clothoidA, -1);
        % horizontal segment
        track = mbc_straight_create(track, a1 - 2 * cornerSize - width, width);
        % corner
        track = create_corner_curve90(track, width, clothoidA, -1);
        % vertical segment
        track = mbc_straight_create(track, a2 - 2 * cornerSize - width, width);
        % corner
        track = create_corner_curve90(track, width, clothoidA, -1);
        % horizontal segment
        track = mbc_straight_create(track, a1 - 2 * cornerSize - width, width);
        % corner
        track = create_corner_curve90(track, width, clothoidA, -1);
        % last vertical segment
        track = mbc_straight_create(track, 0.5 * a2 - cornerSize - 0.5 * width, width);
    case 2
        %% Create race track
        width = 250e-3; % track width
        clothoidA = 8; % clothoid parameter
        turn = 1; % clock / counterclock wise
        % determine 90° corner size
        track = mbc_track_create(0, 0, pi/2); % dummy track
        track = create_corner_curve90(track, width, clothoidA, -1); % dummy corner
        cornerSize90 = track.points{end}.s1;
        % determine 180° corner size
        track = mbc_track_create(0, 0, pi/2); % dummy track
        track = create_corner_curve180(track, width, clothoidA, -1); % dummy corner
        cornerSize180_1 = track.points{end}.s1;
        cornerSize180_2 = track.points{3}.s2;
        % real track
        track = mbc_track_create(a1boundary + 0.5 * width, 0.5 * a2total, -pi/2);
        track = mbc_straight_create(track, 0.5 * a2 - cornerSize180_2 - 0.5 * width, width);
        track = create_corner_curve180(track, width, clothoidA, turn);
        track = create_corner_curve180(track, width, clothoidA, -turn);
        track = create_corner_curve90(track, width, clothoidA, turn);
        track = mbc_straight_create(track, a1 - 2 * cornerSize180_1 - 2 * cornerSize90 - width, width);
        track = create_corner_curve90(track, width, clothoidA, turn);
        track = mbc_straight_create(track, a2 - cornerSize90 - cornerSize180_2 - width, width);
        track = create_corner_curve90(track, width, clothoidA, turn);
        track = mbc_straight_create(track, a1 - 2 * cornerSize90 - width, width);
        track = create_corner_curve90(track, width, clothoidA, turn);
        track = mbc_straight_create(track, 0.5 * a2 - cornerSize90 - 0.5 * width, width);
end

%% discretize and display track
track = mbc_track_display(track, 0.1, [ 0 a1total 0 a2total ]);
end

%% auxiliary functions to create clothoid-based corner curve
function track = create_corner_curve90(track, width, a, turn)
track = mbc_clothoid_create(track, a, turn * pi / 4, width, 0);
track = mbc_clothoid_create(track, a, turn * pi / 4, width, 1);
end

function track = create_corner_curve180(track, width, a, turn)
track = mbc_clothoid_create(track, a, turn * pi / 4, width, 0);
r = 1 / ((track.points{end}.x - track.points{end-1}.x) * a);
track = mbc_circle_create(track, r, turn * pi / 4, width);
track = mbc_circle_create(track, r, turn * pi / 4, width);
track = mbc_clothoid_create(track, a, turn * pi / 4, width, 1);
end