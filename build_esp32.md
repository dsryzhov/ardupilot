1. start Docker Desktop
2. start Windows Shell
3. start ubuntu in Windows Shell
4. docker start 2d8
5. docker exec -ti 2d8 bash
6. source ./modules/esp_idf/export.sh
7. ./waf configure
8. ./waf configure --board=esp32buzz
9. ./waf rover
10. в windows shell
cd U:\home\dsryzhov\ardupilot\build\esp32buzz\esp-idf_build

esptool.py --chip esp32 -b 921600 --port COM3 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB 0x8000 partition_table/partition-table.bin 0x1000 bootloader/bootloader.bin 0x10000 ardupilot.bin




10. для сборки примера
./waf build --target examples/INS_generic

11 cmd U: cd home cd dsryzhov cd ardupilot
port.bat

12.
ESPBAUD=921600 ESPTOOL_PORT=rfc2217://192.168.1.65:3333 ./waf build --target examples/INS_generic --upload

13. stop in cmd

14. putty COM3 115200


