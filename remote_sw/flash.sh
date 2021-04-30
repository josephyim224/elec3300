#!/bin/bash

clear

rm -rf output
mkdir output

# rebuild library if any argument is provided
if [ "$#" -eq 1 ]; then
    mkdir output/lib

    # include online required components
    declare -a COMPONENTS=("adc1" "awu" "beep" "clk" "exti" "flash" "gpio" "i2c" "itc" "iwdg" "rst" "spi" "tim1" "tim2" "tim4" "uart1" "wwdg")
    for f in "${COMPONENTS[@]}"; do
        echo "pocessing $f"
        path="STM8S_StdPeriph_Driver/src/stm8s_$f.c"
        sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -D STM8S003 -c $path -o output/lib/
        echo
    done

    # https://k1.spdns.de/Develop/Hardware/AVR/mixed%20docs.../doc/sdccman.html/node44.html
    sdcclib STM8S_StdPeriph_Driver.lib ./output/lib/*.rel
    echo

    rm -r output/lib
fi

# compile user library
mkdir output/user
sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -Ilib/inc -D STM8S003 -c lib/src/softi2c.c -o output/user/
echo
sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -Ilib/inc -D STM8S003 -c lib/src/SSD1306.c -o output/user/
echo
sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -Ilib/inc -D STM8S003 -c lib/src/MPU6050.c -o output/user/
echo

sdcclib ./output/user.lib ./output/user/*.rel
echo
rm -r output/user

sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -Ilib/inc -D STM8S003 -c stm8s_it.c -o output/
echo
sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -l ./STM8S_StdPeriph_Driver.lib -l ./output/user.lib -I./ -I./STM8S_StdPeriph_Driver/inc -Ilib/inc -D STM8S003 ./main.c -o output/
echo
stat ./output/main.ihx | egrep 'Size'
packihx ./output/main.ihx >./output/main.hex
stat ./output/main.hex | egrep 'Size'
echo
stm8flash -c stlinkv2 -p stm8s003f3 -s flash -w ./output/main.ihx
