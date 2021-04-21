# Setup instructions for Crazyflie with audio deck

## flash EEPROM

git clone https://github.com/bitcraze/crazyflie2-nrf-firmware

cd crazyflie2-nrf-firmware

make BLE=0

make BLE=0 cload

git clone https://github.com/bitcrazye/crazyflie-lib-python

cd crazyflie-lib-python

- Modify the write-ow.py example to fit your data, run it, for instance:

```python
PID = 0x04 ## needs to match with driver.
NAME_ID = 'Buzzer Deck'
REV_ID = 'V3'
```

- You can read back the memory with the read-ow.py example
- If you want bluetooth back you can compile again the nrf firmware (make clean all) or flash the latest released firmware package.
