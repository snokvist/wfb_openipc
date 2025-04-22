socat -u UDP-RECV:8000 -
nc -ulp 8000

i2cdetect -y -r 1 

For check out the IMU id. (68)


![image](https://github.com/user-attachments/assets/95c4146d-cee9-4c5a-b2e7-c451eae487de)
Via i2c1, configuration imu (BMI270), then read data.


https://github.com/CoRoLab-Berlin/bmi270_c/blob/main/example/main.c
