# ESP8266 Flashing

# Download esptool

# Download correct flashing files (under ESP8266 Flashing Files)

To flash firmware run the following prompts:

```bash
python -m esptool --port COM9 --baud 19200 write-flash ^
--flash-mode dio ^
--flash-size 4MB ^
0x00000 boot_v1.7.bin ^
0x01000 user1.1024.new.2.bin ^
0x81000 user2.1024.new.2.bin ^
0x3FC000 esp_init_data_default.bin ^
0x3FE000 blank.bin
