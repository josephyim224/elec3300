#!/bin/bash

rm -rf output
mkdir output

# rebuild library if any argument is provided
if [ "$#" -eq 1 ]; then
    # include online required components
    declare -a COMPONENTS=("adc1" "awu" "beep" "clk" "exti" "flash" "gpio" "i2c" "itc" "iwdg" "rst" "spi" "tim1" "tim2" "tim4" "uart1" "wwdg")
    for f in "${COMPONENTS[@]}"; do
        echo "pocessing $f"
        path="STM8S_StdPeriph_Driver/src/stm8s_$f.c"
        sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -D STM8S003 -c $path -o output/
        echo
    done
    sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -D STM8S003 -c stm8s_it.c -o output/
    echo
    sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -I./ -ISTM8S_StdPeriph_Driver/inc -D STM8S003 -c main.c -o output/
    echo
    sdcc -mstm8 --opt-code-size --std-sdcc11 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -D STM8S003 ./output/main.rel ./output/stm8s*.rel -o output/
    echo

    rm STM8S_StdPeriph_Driver.lib

    # https://k1.spdns.de/Develop/Hardware/AVR/mixed%20docs.../doc/sdccman.html/node44.html
    sdcclib STM8S_StdPeriph_Driver.lib ./output/*.rel
fi

sdcc -lstm8 -mstm8 --opt-code-size --std-sdcc99 --nogcse --all-callee-saves --debug --verbose --stack-auto --fverbose-asm --float-reent --no-peep -l ./STM8S_StdPeriph_Driver.lib -I./ -I./STM8S_StdPeriph_Driver/inc -D STM8S003 ./main.c -o output/
stat ./output/main.ihx
stm8flash -c stlinkv2 -p stm8s003f3 -s flash -w ./output/main.ihx
