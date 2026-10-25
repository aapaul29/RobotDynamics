% START_EXAM  Run this ONCE at the very beginning of the exam.
%
%   It puts the exam folder on the MATLAB path and starts the (disclosed)
%   exam-integrity logger. Running it more than once is harmless: it will
%   not start a second logger.
%
%   When you are finished, run  finalize_submission  to stop the logger and
%   build the zip you upload to Moodle.

thisfolder = fileparts(mfilename('fullpath'));
addpath(genpath(thisfolder));
cd(thisfolder);

audit_engine('start');
