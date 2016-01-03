{$ifdef NDS_INTERFACE}

type
  FifoMessageType = integer;
const
<<<<<<< HEAD
  SOUND_PLAY_MESSAGE        : FifoMessageType = $1234;
  SOUND_PSG_MESSAGE         : FifoMessageType = $1235;
  SOUND_NOISE_MESSAGE       : FifoMessageType = $1236;
  MIC_RECORD_MESSAGE        : FifoMessageType = $1237;
  MIC_BUFFER_FULL_MESSAGE   : FifoMessageType = $1238;
  SYS_INPUT_MESSAGE         : FifoMessageType = $1239;
	SDMMC_SD_READ_SECTORS     : FifoMessageType = $123A;
	SDMMC_SD_WRITE_SECTORS    : FifoMessageType = $123B;
=======
  SOUND_PLAY_MESSAGE      : FifoMessageType = $1234;
  SOUND_PSG_MESSAGE       : FifoMessageType = $1235;
  SOUND_NOISE_MESSAGE     : FifoMessageType = $1236;
  MIC_RECORD_MESSAGE      : FifoMessageType = $1237;
  MIC_BUFFER_FULL_MESSAGE : FifoMessageType = $1238;
  SYS_INPUT_MESSAGE       : FifoMessageType = $1239;
	SYS_SD_READ_SECTORS     : FifoMessageType = $1240;
	SYS_SD_WRITE_SECTORS    : FifoMessageType = $1241;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
{$PACKRECORDS 4} 
  FifoMessage = record
    _type: cuint16;
    case integer of
      0: (
        SundPlay: record
          _type: cuint16;
          data: pointer;
<<<<<<< HEAD
          dataSize: cuint16;
          loopPoint: cuint16;
=======
          loopPoint: cuint16;
          dataSize: cuint16;
>>>>>>> origin/fixes_2.4
          freq: cuint16;
          volume: cuint8;
          pan: cuint8;
          loop: cbool;
          format: cuint8;
        end;
      );
      1: (
        SoundPsg: record
          freq: cuint16;
          dutyCycle: cuint8;
          volume: cuint8;
          pan: cuint8;
<<<<<<< HEAD
          channel: cuint8;
=======
>>>>>>> origin/fixes_2.4
        end;
      );
      2: (
        MicRecord: record
          buffer: pointer;
          bufferLength: cuint32;
          freq: cuint16;
          format: cuint8;
        end;
      );
      3: (
        MicBufferFull: record
          buffer: pointer;
          length: cuint32;
        end;
      );
      4: (
        SystemInput: record
          touch: touchPosition;
          keys: cuint16;
        end;
      );
<<<<<<< HEAD
      5: (
        sdParams: record
          buffer: pointer;
			    startsector: cuint32;
			    numsectors: cuint32;
			  end;
		  );
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      6: (
        fwParams: record
          buffer: pointer;
          address: cuint32;
          length: cuint32;
        end;      
      );  
        
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  end;
  TFifoMessage = FifoMessage;
  PFifoMessage = ^FifoMessage;

{$PACKRECORDS C}   
{$endif NDS_INTERFACE}

