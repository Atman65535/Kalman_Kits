~~~json
[env:genericSTM32F407VET6]
platform = ststm32
board = genericSTM32F407VET6
framework = stm32cube
monitor_speed = 115200
monitor_filters = log2file
~~~
Must modify monitor speed manually.

~~~sh
pio device monitor --baud 57600
~~~

|参数|功能|
|---|---|
|`-p / --port`|指定串口|
|`-b / --baud`|指定波特率|
|`--echo`|是否回显输入字符|
|`--raw`|原始模式，不处理换行符等|
最后还是使用了sprintf搭配串口传输获得的数据。