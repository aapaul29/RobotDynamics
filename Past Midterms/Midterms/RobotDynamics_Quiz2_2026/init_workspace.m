% Set up Midterm 2 executable folders without traversing the audit vault.
exercisefolder = fileparts(mfilename('fullpath'));
exercisefolder = bootstrap_exam(exercisefolder, true);
clear exercisefolder;

function root = bootstrap_exam(root, validate_course)
    folder = fullfile(root, 'exam_logger');
    expected = fullfile(folder, 'exam_setup.p');
    if ~isfile(expected), expected = fullfile(folder, 'exam_setup.m'); end
    if ~isfile(expected)
        shared = fullfile(fileparts(fileparts(root)), 'exam_logger');
        if isfile(fullfile(root, 'logger_config.m')) && ...
                isfile(fullfile(shared, 'setup_instructor_exam.m'))
            cleanup = onCleanup(@() cd(root)); %#ok<NASGU>
            cd(root); config = logger_config();
            cd(shared);
            root = setup_instructor_exam(root, config, validate_course);
            return;
        end
        error('exam:missingSetup', 'Exam setup is missing. Re-extract this folder or contact a TA.');
    end
    selected = which('exam_setup');
    if ispc, same = strcmpi(selected, expected); else, same = strcmp(selected, expected); end
    if ~same
        cleanup = onCleanup(@() cd(root)); %#ok<NASGU>
        cd(folder);
    end
    root = exam_setup(validate_course);
end
