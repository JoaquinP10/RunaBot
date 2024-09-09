# RunaBot
Repositorio referente al avances y diseño de robot humanoide RunaBot enfocado a competencias internacionales como LARC.

## Concerning info about esp32 directory
In esp32/examples/ttgo_demo there is all progress in esp32cam in ESP-IDF IDE. However, the big issue for providing the code is that you'll have to make additional configuration in order to access the program, since the root library was made for Linux.

Tutotutorial in progress...

You can get an idea seeing these resources: 


- https://www.youtube.com/watch?v=y3RcDDzeXDM&t=2s: shows information about how to compile and flash esp32cam using usb to ttl driver, since cam does not have any usb input.

- https://www.youtube.com/watch?v=7qPIRBY6C8c: This video shows basically all the problems I have to dealt with. The best solution is to use Docker to build the program and use your local PC to flash. However, he is using as program made for ttgo, which is another type of cam

- https://github.com/joachimBurket/esp32-opencv/issues/2: Joachimb Burket repo contains all data about esp32cam and opencv. In this issue, he explains better the procedure to use his programs on Windows using Docker.

