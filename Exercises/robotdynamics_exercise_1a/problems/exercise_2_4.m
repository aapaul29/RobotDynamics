% 2.4. Endeffector Position for all angles = pi / 6
q = pi/6 * ones(6, 1);
endEffectorPosition = jointToPosition(q);
loadviz;
abbRobot.setJointPositions(q);
display(endEffectorPosition);