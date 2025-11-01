%% Overall clock rate, for use on AD937x, always 122.88e6
% This is merely clock rate, AD937x must run at this clock rate, its Rx
% sampling rate is the same but Tx sampling rate is twice.
SAMPLE_RATE = 122.88e6;
SAMPLE_TIME = 1 / SAMPLE_RATE;
SAMPLE_PER_US = SAMPLE_RATE / 1e6;

%% Create RFID Config Object
% BLF(Hz) and Tari(us) combination we use：
%  - 40e3, 23.75
%  - 80e3, 23.75
%  - 120e3, 18
%  - 160e3, 13.5
%  - 200e3, 11
%  - 320e3, 7
%  - 640e3, 6.25
BLF = 40e3;
Tari = 23.75;
TariRatio = 2;
RFID = RFIDConfig(BLF, Tari, TariRatio, SAMPLE_RATE);

%% QuinID Config
CenterFreq  = 915.e6;
OperateFreq = [902.875e6, 908.42e6, 916.5e6, 921.42e6, 926e6];
NCO = gen_nco_config(SAMPLE_RATE, OperateFreq - CenterFreq);

%% Rx DDC conversion filters
% The Rx rate goes through a decimation, followed by an transition, in
%  order to ensure the final rate is [8 * BLF].
%  With 2 FM0 symbols to represent 1 bit and 4 points to represent 1 symbol.
% Directly use dsp.SampleRateConverter system object, for it contains
% similar functionality with designMultistageDecimator and is able to do
% fractional rate conversion.
SRC = dsp.SampleRateConverter( ...
    'InputSampleRate', SAMPLE_RATE, ...
    'OutputSampleRate', 8 * BLF, ...
    'Bandwidth', 6 * BLF, ...
    'StopbandAttenuation', 60);
FILT = getFilters(SRC);
% Use these functions to view the filter
% 1. visualizeFilterStages(SRC);
% 2. fvtool(FILT, 'Fs', SAMPLE_RATE);

%% Tx filters
% Note that tx in AD937x should feed 2 samples every clock, making the
% actual tx rate equals 2 * clock_rate. But since the model runs at clock
% rate and our NCO also works at clock rate, we use a TX_RC filter in
% clock rate and add a interpolation afterwards, which has a factor of 2.
TX_RC = gen_raised_cosine_filter( ...
    110, ...              % TX RC order
    SAMPLE_RATE, ...      % TX sample rate
    500e3);               % TX RC bandwidth
TX_INTER = designMultirateFIR(2, 1, 0.25, 60,"SystemObject", true);
% Use this function to view the filter
%  - fvtool(TX_RC, 'Fs', SAMPLE_RATE);
%  - fvtool(TX_INTER, 'Fs', SAMPLE_RATE * 2);

%% Model Loopback filter
% This loopback filter decimates 245.76Msps to 122.88Msps for loopback in
% simulation.
LPBACK_DECI = designMultirateFIR(1, 2, 0.25, 60,"SystemObject", true);
% Use this function to view the filter
%  - fvtool(LPBACK_DECI, 'Fs', SAMPLE_RATE * 2);

%% Other Model parameters
ALIGN_SIG_LEN = 5000;

%% Show Configuration
fprintf("Current Config:\n");
fprintf(" - AD937x clock rate: %.02fMsps\n", SAMPLE_RATE/1e6);
fprintf(" - AD937x's rx sample rate equals to clock rate.\n");
fprintf(" -          tx sample rate is 2 * clock rate but still runs at clock rate.\n");
fprintf(" - RFID BLF setting: %dKHz\n", BLF/1e3);
fprintf(" - RFID DivRatio: %d\n", RFID.DivRatio);
fprintf(" - Rx filter chain:\n %s\n", info(SRC));
fprintf(" - Rx filter output Delay: %d samples.\n", FILT.outputDelay());
fprintf(" - Tx filter output Delay: %d samples.\n", TX_RC.outputDelay());
fprintf(" - Decoding should finish between %d to %d us.\n", ...
    RFID.Dur.T2_Min_D, RFID.Dur.T2_Max_D);

%% Utility function
function rc_filter = gen_raised_cosine_filter(N, Fs, Fc)
    % N: Order
    TM   = 'Rolloff';  % Transition Mode
    R    = 1;          % Rolloff
    DT   = 'Normal';   % Design Type
    Beta = 0.5;        % Window Parameter
    
    % Create the window vector for the design algorithm.
    win = kaiser(N+1, Beta);
    
    % Calculate the coefficients using the FIR1 function.
    b  = firrcos(N, Fc/(Fs/2), R, 2, TM, DT, [], win);
    rc_filter = dsp.FIRFilter('Numerator', b);
end

function nco = gen_nco_config(sample_rate, freqs)
    % Parameters
    Deltaf = 20;          % Frequency resolution (Hz)
    SFDR = 110;           % Spurious free dynamic range (dB)
    Ts = 1 / sample_rate; % Sample period (s)

    % Design general
    nco = struct();
    nco.N = ceil(log2(1/(Ts*Deltaf)));
    nco.Q = ceil((SFDR - 12)/6);
    nco.ditherBits = nco.N - nco.Q;
    % Design phase increment
    nco.phIncr = round(freqs*2^nco.N*Ts);
end