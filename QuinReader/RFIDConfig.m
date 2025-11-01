%% Note
% This file provides supported RFID operation basics
%  - Bit level, we use FM0 coding, pilot is added when BLF > 320kHz
%  - Command level, we support sending Query, QueryRep and ACK commands
%  - Rate level, all rate-related parameters are set accroding to input
%  - PHY level:
%     1) RTCal is set to its maximum (3 * Tari)
%     2) DR is set accroding to BLF, DR=64/3 is used when BLF is larger than 320kHz
%     3) PW is set to 0.4 * Tari
%     4) Downlink bit 1 is set to be 2 * Tari
classdef RFIDConfig < handle
    properties
        % Bit number constants %
        TAG_PREAMBLE_BITNUM  = 6;   % Number of Tag to reader preamble bits
        DUMMY_BITNUM         = 1;   % Dummy bit at the end of the packet
        RN16_BITNUM          = 16;  % RN16 has a dummy bit at the end
        EPC_BITNUM           = 128; % Length of EPC *without* Preamble and dummy.
        TOTAL_BITNUM         = 6 + 16 + 96 + 16 + 1; % Preamble + PC + EPC + CRC16 + Dummy = 6 + 16 + 96 + 16 + 1 = 135
        PILOT_NUM            = 12;  % Pilot number, containing 12 FM0 zeros, symbols are [1 0]

        % EPC Parameters %
        TAG_BLF
        DivRatio
        PilotEN
        FrT
        Dur           % duration object

        % Tx Rx Objects %
        Tx      % Tx uses CLK_RATE as sample rate
        Rx      % Rx uses its own rate but runs at CLK rate
    end
    
    methods
        function obj = RFIDConfig(TAG_BLF, Tari_D, Tari_Ratio, TX_SAMPLE_RATE)
            %% ---------------------------------- Basic ------------------------------------
            obj.Tx = struct();
            obj.Rx = struct();
            obj.Dur = struct();

            Simulink.defineIntEnumType('ReaderCommand', {'Query', 'QueryRep', 'ACK'}, [0; 1; 2], ...
                'Description', 'Type of command reader transmits', ...
                'DefaultValue', 'Query', ...
	            'AddClassNameToEnumNames', true, ...
                'StorageType', 'int8');
            Simulink.defineIntEnumType('ReaderReceive', {'RN16', 'EPC'}, [0; 1], ...
                'Description', 'Type of backscatter packets reader receives', ...
                'DefaultValue', 'RN16', ...
	            'AddClassNameToEnumNames', true, ...
                'StorageType', 'int8');

            %% ---------------------------------- Tx Bits -----------------------------------
            % All defined inside model
            % Query command = Code (1000) + DR + M + TRext + Sel + Session + Target + Q + CRC
            %  - DR: 1 bit, 0->[DR=8], 1->[DR=64/3]
            %  - M: 2 bits, 00 for FM0
            %  - TRext: 1 bit, use pilot or not
            %  - Sel: 2 bits, tag choose, usually 00
            %  - Session: 2 bits, S0 to S3
            %  - Target: 1 bits, 0->[A], 1->[B]
            %  - Q: 4 bits
            %  - CRC: 5 bits
            % QueryRep command = Code (00) + Session (same with Query)
            % ACK command = Code (01) + RN16 (echoed, 16 bits)
            % NAK command = Code (11000000)
            % QueryAdjust command = Code (1001) + Session (same with Query) + UpDn (3 bits)
            %  - UpDn: `110` Increment by 1, `000` Unchanged, `011` Decrement by 1

            %% ------------------------------ Duration (All in us) ---------------------------
            % Basic parameter input
            if any(Tari_D < 6.25) || any(Tari_D > 25)
                error('Tari should be in range [6.25, 25] us.')
            end
            if any(TAG_BLF < 40e3) || any(TAG_BLF > 640e3)
                error('BLF should be in range [40e3, 640e3] Hz.')
            end
            obj.Dur.Tari_D  = Tari_D;
            obj.TAG_BLF = TAG_BLF;

            % Set DivRatio, can be 8 or 64/3
            obj.DivRatio = (TAG_BLF <= 320e3) * 8 + (TAG_BLF > 320e3) * 64/3;

            % Set Pilot, currently only enable at 640kHz
            obj.PilotEN = TAG_BLF > 320e3;
            
            % Set FrT
            obj.FrT = arrayfun(@(x) obj.determineFrT(x), TAG_BLF);

            % Reader to tag basic duration %
            obj.Dur.PW_D    = max(0.4 * obj.Dur.Tari_D, 2);   % MAX(0.265*Tari, 2) <= PW <= 0.525Tari
            obj.Dur.DATA0_D = obj.Dur.Tari_D;                 % data0 is one Tari
            if any(Tari_Ratio < 1.5) || any(Tari_Ratio > 2.0) % data1 is 1.5 ~ 2 times Tari
                error('Tx data 1 must be 1.5~2.0 times Tari (data 0)')
            end
            obj.Dur.DATA1_D = obj.Dur.Tari_D * Tari_Ratio;     
            
            % Tag to reader basic duration %
            obj.Dur.Tpri_D  = 1e6 ./ obj.TAG_BLF; % period of an FM0 symbol, not always an int. BLF = 1 / Tpri

            % Reader to tag preamble and sync duration %
            obj.Dur.DELIM_D = 12.5;                                % A preamble comprises a fixed-length start delimiter 12.5us +/-5%
            obj.Dur.RTCAL_D = obj.Dur.DATA0_D + obj.Dur.DATA1_D;   % RTCAL is data0 + data1
            obj.Dur.TRCAL_D = obj.DivRatio * 1e6 ./ obj.TAG_BLF;    % BLF = DR / TRCAL
            if any(obj.Dur.TRCAL_D < 1.1 * obj.Dur.RTCAL_D) || ...
                   any(obj.Dur.TRCAL_D > 3 * obj.Dur.RTCAL_D)
                error('TRCAL not in the range of [1.1*RTCAL, 3*RTCAL], check input Tari')
            end
            
            % Reading protocol duration config %
            obj.Dur.CW_START_D = 50;        % Carrier wave duration at the initialization
            obj.Dur.T1_D     = max(obj.Dur.RTCAL_D, 10 * obj.Dur.Tpri_D);  % Time from Interrogator transmission to Tag response (10 * Tpri)
            obj.Dur.T1_Min_D = obj.Dur.T1_D .* (1 - obj.FrT) - 2; % Min T1
            obj.Dur.T1_Max_D = obj.Dur.T1_D .* (1 + obj.FrT) + 2; % Max T1
            obj.Dur.T2_Min_D = 3 * obj.Dur.Tpri_D; % Time from Tag response to Interrogator transmission. Min=3*Tpri and Max=20*Tpri
            obj.Dur.T2_Max_D = 20 * obj.Dur.Tpri_D;
            % Extra duration config %
            if obj.PilotEN
                obj.Dur.Pilot_D = obj.Dur.Tpri_D * obj.PILOT_NUM;
            else
                obj.Dur.Pilot_D = 0;
            end
            obj.Dur.Pilot_Min_D = obj.Dur.Pilot_D .* (1 - obj.FrT);
            obj.Dur.Pilot_Max_D = obj.Dur.Pilot_D .* (1 + obj.FrT);

            %% ---------------------------------- Tx Duration Config -----------------------------------
            obj.Tx.QUERY_DR = TAG_BLF > 320e3;
            obj.Tx.TRext    = obj.PilotEN;
            obj.Tx.SAMPLE_PER_US = TX_SAMPLE_RATE / 1e6;
            % Reader Transmit Hi Lo state sample %
            obj.Tx.DELIM_SAMPLE    = obj.Dur.DELIM_D * obj.Tx.SAMPLE_PER_US;
            obj.Tx.CW_START_SAMPLE = obj.Dur.CW_START_D * obj.Tx.SAMPLE_PER_US;
            obj.Tx.PW_SAMPLE       = round(obj.Dur.PW_D * obj.Tx.SAMPLE_PER_US);
            obj.Tx.DATA0_HI_SAMPLE = ceil(obj.Dur.DATA0_D * obj.Tx.SAMPLE_PER_US - obj.Tx.PW_SAMPLE);
            obj.Tx.DATA1_HI_SAMPLE = ceil(obj.Dur.DATA1_D * obj.Tx.SAMPLE_PER_US - obj.Tx.PW_SAMPLE);
            obj.Tx.RTCAL_HI_SAMPLE = ceil(obj.Dur.RTCAL_D * obj.Tx.SAMPLE_PER_US - obj.Tx.PW_SAMPLE);
            obj.Tx.TRCAL_HI_SAMPLE = round(obj.Dur.TRCAL_D * obj.Tx.SAMPLE_PER_US) - obj.Tx.PW_SAMPLE;

            %% -------------------------------------- Rx Config ---------------------------------------
            % FM0 encoding preamble sequences (tag to reader), this preamble stores half bits, every two correspond one actual preamble FM0 bit %
            %  In our implementation, time sync happens before channel equalization, and received signal are represented as "low" and "high". 
            %  The "low" here is the tag state when not backscattering, therefore is close to zero after DC removal, which is the 0 in FM0 
            %  preamble.
            obj.Rx.FM0_PREAMBLE = [1,1,-1,1,-1,-1,1,-1,-1,-1,1,1]';

            % Rx requires 4 samples per symbol (half bit) %
            obj.Rx.SAMPLE_PER_SYMBOL = 4;
            obj.Rx.SAMPLE_RATE = obj.TAG_BLF * obj.Rx.SAMPLE_PER_SYMBOL * 2; % 4 sample point per symbol, so 8 per bit
            obj.Rx.SAMPLE_PER_US = obj.Rx.SAMPLE_RATE / 1e6;

            % Rx Matching Filter %
            obj.Rx.FIR_MATCH_COFF = ones(1, obj.Rx.SAMPLE_PER_SYMBOL) / obj.Rx.SAMPLE_PER_SYMBOL;

            % DC Removal CW estimation window (before each RN16 and EPC) %
            obj.Rx.DC_WINDOW_D    = obj.Dur.T1_Min_D / 2;
            obj.Rx.DC_WINDOW_S    = floor(obj.Rx.DC_WINDOW_D .* obj.Rx.SAMPLE_PER_US);
            obj.Rx.DC_WINDOW_GAIN = 1 ./ obj.Rx.DC_WINDOW_S;

            % Preamble search sample %
            obj.Rx.T1_Min_SAMPLE = floor(obj.Dur.T1_Min_D .* obj.Rx.SAMPLE_PER_US);
            obj.Rx.T1_Max_SAMPLE =  ceil(obj.Dur.T1_Max_D .* obj.Rx.SAMPLE_PER_US);
            obj.Rx.Pilot_SAMPLE     = round(obj.Dur.Pilot_D .* obj.Rx.SAMPLE_PER_US);
            obj.Rx.Pilot_Min_SAMPLE = floor(obj.Dur.Pilot_Min_D .* obj.Rx.SAMPLE_PER_US);
            obj.Rx.Pilot_Max_SAMPLE =  ceil(obj.Dur.Pilot_Max_D .* obj.Rx.SAMPLE_PER_US);
            % search length (add another 5 for FIR Filter induced latency, a.k.a., group delay,
            %   and tolerance)
            obj.Rx.SEARCH_SAMPLE = (obj.Rx.T1_Max_SAMPLE - obj.Rx.T1_Min_SAMPLE) + ...
                (obj.Rx.Pilot_Max_SAMPLE - obj.Rx.Pilot_Min_SAMPLE) + 5;

            % Receive preamble %
            obj.Rx.PREAMBLE_SIG         = repelem(obj.Rx.FM0_PREAMBLE, obj.Rx.SAMPLE_PER_SYMBOL)';
            obj.Rx.PREAMBLE_SAMPLE      = length(obj.Rx.PREAMBLE_SIG);
            obj.Rx.PREAMBLE_NORM_FACTOR = 1 / obj.Rx.PREAMBLE_SAMPLE;

            % Channel equalizer %
            % hard set now, this is considering the clock variation of the
            % tag within preamble.
            obj.Rx.EQUALIZER = zeros(1, obj.Rx.PREAMBLE_SAMPLE);
            obj.Rx.EQUALIZER(2:7) = 1;
            obj.Rx.EQUALIZER_NORM_FACTOR = 1 / sum(obj.Rx.EQUALIZER);

            % Fine Time Synchronization %
            % currently left peek is 2 right peek is 3
            % method 1: xcorr with first two ones in the preamble.
            % method 2: use derivative and get the maximum.
            obj.Rx.FINE_TS_LEFT_PEEK = obj.Rx.SAMPLE_PER_SYMBOL - 2;
            obj.Rx.FINE_TS_PEEK_LEN = 2 * obj.Rx.SAMPLE_PER_SYMBOL - 2;
            obj.Rx.FINE_TS_XCORR = repelem([1 1], obj.Rx.SAMPLE_PER_SYMBOL);
            obj.Rx.FINE_TS_NORM_FACTOR = 1 / sum(obj.Rx.FINE_TS_XCORR);
        end
    end

    methods (Access = private)
        function FrT = determineFrT(obj, BLF)
            if BLF < 40e3
                error('BLF too small, at least 40KHz')
            elseif BLF < 107e3
                FrT = 0.04;
            elseif BLF <= 160e3
                FrT = 0.07;
            elseif BLF <= 256e3
                FrT = 0.10;
            elseif BLF <  320e3
                FrT = 0.12;
            elseif BLF == 320e3
                FrT = 0.10;
            elseif BLF <  640e3
                FrT = 0.22;
            elseif BLF == 640e3
                FrT = 0.15;
            else
                error('BLF too large, at most 640KHz')
            end
        end
    end
end
