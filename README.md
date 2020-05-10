This is an Arduino sketch that uses a HuskyLens camera to drive a rover, following a line of tape.

See it in action here: https://youtu.be/hXYdbQ042MY

Details here: https://diyrobocars.com/?p=4139

It requires the following:

Hardware:

--HuskyLens: https://www.dfrobot.com/product-1922.html

--A Teensy LC: https://www.pjrc.com/store/teensylc.html

--A PCB to mount the Teensy on and break out necessary pins: https://oshpark.com/shared_projects/vUhzNyaW

--A RC car/rover chassis of some sort. The DonkeyCar ones will work fine: https://amzn.to/2KsYF1e

--A 3D-printed camera mount: https://www.shapeways.com/product/4KUSU43S3/openmv-basic-mount

Software (add as Arduino libraries in the Arduino IDE)
--The HuskyLens Arduino library: https://codeload.github.com/HuskyLens/HUSKYLENSArduino/zip/master

--The AutoPID Arduino library: https://r-downing.github.io/AutoPID/
