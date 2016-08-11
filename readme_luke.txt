Attempt to build a stup-up (3v-4.2V in, 5V out) on stm32F030c4p6. But with a lot of steps in between, it's mostly for the learning experience and the goal might change later on too. (To a 3ch 0-60mA output, led driver).

Currently: Getting current consumption of MPU-6050 down in a sleep mode where it still can trigger motion interrupt. Might not get it below 240 uA, at lower sample rates current consumption goes down but also sensentivity. Maybe should focus on MCU sleep mode first.
