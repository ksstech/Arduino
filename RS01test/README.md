# RS01test
Irmacos RS01 controller test firmware

/* Notes:
1. 80Mhz, enabling PSRAM, crash
2. 80Mhz, disabling PSRAM, OK
3. 160Mhz, remove delay(msDELAY) in checkSerials() & checkSerialsInt(), OK
4. 160Mhz, remove RS485 adapter & cable, OK
5. 160Mhz, reduce fixMessage delay 200->100mS, OK
6. 160Mhz, reduce fixMessage delay 100->20mS, OK
7. 240Mhz, reduce fixMessage delay 20->10mS, OK
8. Reduce fixMessage delay 10->2mS, OK
9. Enable PSRAM, crash
10. Increase fixMessage delay 2->200mS, crash
11. Disable PSRAM, reduce fixMessage delay 200->2mS, OK
12. Disable fixMessage delay completely, OK

*/
