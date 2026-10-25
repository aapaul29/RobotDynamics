% FINALIZE_SUBMISSION  Run this ONCE when the exam time is up.
%
%   It (1) stops the integrity logger and flushes/encrypts your command
%   history, (2) asks for your ETH student (legi) number and name, and
%   (3) zips the ENTIRE exam folder - including the audit_vault folder - into
%   ETHStudentID_StudentName.zip (e.g. 01234567_JohnDoe.zip), which you then
%   upload to Moodle. You do not need to add any file by hand.

thisfolder = fileparts(mfilename('fullpath'));
addpath(genpath(thisfolder));
cd(thisfolder);

%% 1. Stop logging cleanly (safe even if it was never started / already stopped)
try
    audit_engine('stop');
catch
    warning('Could not stop the integrity logger cleanly; continuing to package.');
end

%% 2. Ask for the legi number and name
legi = ''; name = '';
try
    legi = strtrim(input('Enter your ETH student (legi) number: ', 's'));
    name = strtrim(input('Enter your name (e.g. John Doe): ', 's'));
catch
    % Non-interactive session: fall back to a timestamp below.
end
legi = regexprep(legi, '[^0-9]', '');          % digits only
name = regexprep(name, '[^A-Za-z0-9]', '');    % letters/digits, no spaces
if isempty(legi), legi = datestr(now, 'yyyymmdd'); end      %#ok<TNOW1,DATST>
if isempty(name), name = datestr(now, 'HHMMSS'); end        %#ok<TNOW1,DATST>

%% 3. Check the audit vault exists and is non-empty
vault = fullfile(thisfolder, 'audit_vault');
nframes = numel(dir(fullfile(vault, 'frame_*.dat')));
if ~exist(vault, 'dir') || nframes == 0
    warning(['No integrity samples were found in %s.\n' ...
             '         Did you run start_exam at the beginning? Your submission\n' ...
             '         will still be built, but please tell a proctor now.'], vault);
else
    fprintf('Integrity vault OK: %d sample(s) found.\n', nframes);
end

%% 4. Zip the ENTIRE folder (everything except previous submission zips)
zipname = fullfile(thisfolder, sprintf('%s_%s.zip', legi, name));
if exist(zipname, 'file'), delete(zipname); end

entries = dir(thisfolder);
contents = {};
for k = 1:numel(entries)
    nm = entries(k).name;
    if strcmp(nm, '.') || strcmp(nm, '..'), continue; end
    if ~isempty(regexpi(nm, '\.zip$')), continue; end   % skip any .zip files
    contents{end+1} = nm; %#ok<SAGROW>
end

zip(zipname, contents, thisfolder);

%% 5. Verify the zip really contains the audit folder
try
    listed = unzip_list(zipname);
    has_audit = any(contains(listed, ['audit_vault' filesep]) | contains(listed, 'audit_vault/'));
catch
    has_audit = false;
end

fprintf('\n==============================================================\n');
fprintf('  SUBMISSION BUILT\n');
fprintf('  File:  %s\n', zipname);
if has_audit
    fprintf('  Audit folder included:  YES\n');
else
    fprintf('  Audit folder included:  *** NO - tell a proctor ***\n');
end
fprintf('  >> Upload this single .zip to Moodle under "Midterm Exam 1\n');
fprintf('     Submission". If upload fails, email it from your ETH address\n');
fprintf('     to robotdynamics@leggedrobotics.com\n');
fprintf('==============================================================\n\n');

% --- helper: list the names inside a zip without extracting ---
function names = unzip_list(zipfile)
    zf = java.util.zip.ZipFile(zipfile);
    e  = zf.entries();
    names = {};
    while e.hasMoreElements()
        names{end+1} = char(e.nextElement().getName()); %#ok<SAGROW>
    end
    zf.close();
end
