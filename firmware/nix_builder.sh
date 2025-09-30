source $stdenv/setup

cp --no-preserve=mode -r $src firmware
cd firmware
cmake --preset release
cd build_release
cmake --build .
chmod a-x firmware.elf
arm-none-eabi-strip firmware.elf
mkdir -p $out
cp firmware.{elf,map} $out/
arm-none-eabi-size firmware.elf > $out/firmware.size
arm-none-eabi-objdump -h -S firmware.elf > firmware.list
