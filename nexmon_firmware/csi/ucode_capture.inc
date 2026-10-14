// ARM verifies this receive generation after suspending the MAC.
csi_begin_snapshot:
 mov 0, CSI_PENDING
 mov SPR_TSF_WORD0, CSI_TSF_LOW
 mov SPR_TSF_WORD1, CSI_TSF_HIGH
 xadd CSI_SERIAL, 1, CSI_SERIAL
 mov [0x819], SPARE1
 mov SPARE1, [0xbb0]
 mov [0x81a], SPARE1
 mov SPARE1, [0xbb1]
 mov [0x81b], SPARE1
 mov SPARE1, [0xbb2]
 mov [0x81f], SPARE1
 mov SPARE1, [0xbb3]
 mov [0x879], SPARE1
 mov SPARE1, [0xbb4]
 mov [0x870], SPARE1
 mov SPARE1, [0xbb5]
 mov [0x871], SPARE1
 mov SPARE1, [0xbb6]
 mov [0x872], SPARE1
 mov SPARE1, [0xbb7]
 mov [0x873], SPARE1
 mov SPARE1, [0xbb8]
 mov [0x874], SPARE1
 mov SPARE1, [0xbb9]
 mov [0x875], SPARE1
 mov SPARE1, [0xbba]
 mov CSI_RX_GENERATION, SPARE1
 mov SPARE1, CSI_CAPTURE_GENERATION
 // Publish only after all descriptor fields are written.
 mov 1, CSI_PENDING
 rets
