filename=[
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_12_17.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_14_19.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_15_04.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_17_46.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_19_52.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_25_46.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_28_23.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_28_26.mat';
%%'../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_29_10.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_32_16.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_35_46.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_37_00.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_39_01.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_39_35.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_42_31.mat';
'../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_43_11.mat'
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_51_31.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_53_46.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_14_58_50.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_15_03_05.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_15_05_50.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_15_10_12.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_15_12_32.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_15_16_46.mat';
% '../read CPEV data/CPEV160726/CPEV_Record_2016_07_26_15_25_54.mat'
];
savename=[
% 'trajectory/trajectory_0726141217.mat';
% 'trajectory/trajectory_0726141419.mat';
% 'trajectory/trajectory_0726141504.mat';	% bad
% 'trajectory/trajectory_0726141746.mat';	% bad
% 'trajectory/trajectory_0726141952.mat';
% 'trajectory/trajectory_0726142546.mat';
% 'trajectory/trajectory_0726142823.mat';
% 'trajectory/trajectory_0726142826.mat';	% bad
%%'trajectory/trajectory_0726142910.mat';	
% 'trajectory/trajectory_0726143216.mat';	% bad
% 'trajectory/trajectory_0726143546.mat';
% 'trajectory/trajectory_0726143700.mat';	% bad
% 'trajectory/trajectory_0726143901.mat';
% 'trajectory/trajectory_0726143935.mat';
% 'trajectory/trajectory_0726144231.mat';
'trajectory/trajectory_0726144311.mat';
% 'trajectory/trajectory_0726145131.mat';	% bad
% 'trajectory/trajectory_0726145346.mat';	% bad
% 'trajectory/trajectory_0726145850.mat';
% 'trajectory/trajectory_0726150305.mat';
% 'trajectory/trajectory_0726150550.mat';
% 'trajectory/trajectory_0726151012.mat';
% 'trajectory/trajectory_0726151232.mat';	% bad
% 'trajectory/trajectory_0726151646.mat';
% 'trajectory/trajectory_0726152554.mat'	% bad
];

for nameit=1:1
	fn=filename(nameit,1:end)
	gridMap
	save(savename(nameit,1:end),'wallcloud','traj_data')
end