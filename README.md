### Description
An RTCM3 inject mavros plugin that reads RTCM3 stream from ros topic and forwards it to PX4 firmware.

Currently to RTCM3 stream topic is set to `/rtk_000001/rtcm_stream`



### Installation
```sh
cd ~/ros1_ws/src
git clone https://github.com/ArkadiuszNiemiec/rtcm_inject.git
cd ~/ros1_ws/
catkin build
```