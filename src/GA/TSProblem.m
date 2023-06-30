function varargout = TSProblem(varargin)
    % Initiallize default configuration
    defaultConfig.xy            = 50*rand(200,2);
    defaultConfig.dmat          = [];
    defaultConfig.popSize       = 150;
    defaultConfig.numInter      = 1e4;
    defaultConfig.showProg      = true;
    defaultConfig.showResult    = true;
    defaultConfig.showWaitbar   = true;

    % Interpret user configuration inputs
    if ~nargin
        userConfig = struct();
    elseif isstruct(varargin(l))
        userConfig = varargin(l);
    else
        try
            userConfig = struct(varargin(:));
        catch
            error('Expected inputs are either a structure or parameter/value pairs');
        end
    end

    % Override default configuration with user inputs
    configStruct = get_config(defaultConfig, userConfig);

    % Extract configuration
    xy                  = configStruct.xy;
    dmat                = configStruct.dmat;
    popSize             = configStruct.popSize;
    numInter            = configStruct.numIter;
    showProg            = configStruct.showProg;
    showResult          = configStruct.showResult;
    showWaitbar         = configStruct.showWaitbar;
    if isempty(dmat)
        nPoints = size(xy,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);
    end

    % Verify Inputs
    [N, dims] = size(xy);
    [nr,nc] = size(dmat);