  WARNING!!
  USE --no-stub option for slow unreliable connections!!

  -----------------------------------------
  C:\ESP32\esptool>esptool.py --chip esp32 --port socket://192.168.1.234:24 write_flash 0xe000  C:\Users\TJ\Desktop\tmp\FTP_TMP\boot_app0.bin 0x1000 C:\Users\TJ\Desktop\tmp\FTP_TMP\bootloader_qio_80m.bin 0x10000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.bin 0x8000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.partitions.bin
  esptool.py v3.0-dev
  Serial port socket://192.168.1.234:24
  Connecting.....
  Chip is ESP32-D0WDQ6 (revision 1)
  Features: WiFi, BT, Dual Core, Coding Scheme None
  WARNING: Detected crystal freq 18.43MHz is quite different to normalized freq 26MHz. Unsupported crystal in use?
  Crystal is 26MHz
  MAC: 30:ae:a4:32:c7:10
  Uploading stub...
  Running stub...
  Stub running...
  Configuring flash size...
  Auto-detected Flash size: 4MB
  Compressed 8192 bytes to 47...
  Wrote 8192 bytes (47 compressed) at 0x0000e000 in 0.0 seconds (effective 2048.0 kbit/s)...
  Hash of data verified.
  Compressed 17392 bytes to 11186...
  Wrote 17392 bytes (11186 compressed) at 0x00001000 in 0.5 seconds (effective 294.8 kbit/s)...
  Hash of data verified.
  Compressed 207824 bytes to 105394...
  Wrote 207824 bytes (105394 compressed) at 0x00010000 in 4.3 seconds (effective 382.3 kbit/s)...
  Hash of data verified.
  Compressed 3072 bytes to 128...
  Wrote 3072 bytes (128 compressed) at 0x00008000 in 0.0 seconds (effective 630.2 kbit/s)...
  Hash of data verified.
  
  Leaving...
  Hard resetting via RTS pin...
  
  C:\ESP32\esptool>
  -------------------------------------

# Arduino
Arduino based firmware

Results from testing at various speeds and esptool commands
 set 115200  128000  192000  230400  256000  345600  384000  460800  512000  576000
 get 115942  128000  192771  231884  256000  345665  384038  460929  512000  576057
Tested with 0x20000 read_flash
     0/5     0/5     5/5     0/5     2/5     1/5     0/5     0/5     0/5     0/5
Tested with 0x40000 read_flash
                     3/5
Tested with 0x60000 read_flash
                     1/5             0/10

ESProg module pinout:
=====================
Vcc   GND   IO27  IO26  IO35  IO25  IO17  IO16  IO21  IO22
Vcc   GND   IO32  IO34  IO33  IO32  IO18  IO19  GND   Vcc

            RTS2  TXD2  RXD2  DTR2              SDA   SCL
            RTS1  RXD1  TXD1  DTR1

TEST:
esptool.py --chip esp32 --port socket://192.168.0.28:24 --before default_reset --after hard_reset flash_id 

FORCE RESET:
esptool.py --chip esp32 --port socket://192.168.0.28:24 flash_id

FW UPLOAD:
esptool.py --chip esp32 --port socket://192.168.0.28:24 --before default_reset --after hard_reset write_flash "@flash_args"

FW+BOOT+PART UPLOAD:
esptool.py --chip esp32 --port socket://192.168.0.28:24 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size detect --flash_freq 80m 0x1000 bld_ac01.dbg.2/bootloader/bootloader.bin 0x8000 bld_ac01.dbg.2/partition_table/partition-table.bin 0xd000 bld_ac01.dbg.2/ota_data_initial.bin 0x10000 bld_ac01.dbg.2/be6b89b82590471b82d7b2584c86e665.bin
