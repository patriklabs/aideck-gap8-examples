docker run --rm -v ${PWD}:/module aideck-with-autotiler tools/build/make-example examples/other/wifi-drone-streamer image

cfloader flash examples/other/wifi-drone-streamer/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/80/2M