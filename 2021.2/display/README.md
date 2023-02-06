# ssd1306_rtos_i2c

Projeto para a contrução de um módulo de display para um sistema embarcado de tempo real.

<p>As rotinas com as tarefas de leitura dos dados e envio para o display estão no arquivo <a href="https://github.com/carolinesma/ssd1306_rtos_i2c/blob/main/Core/Src/app_display.c"> app_display.c</a>.<br />As rotinas para criação das filas a funções de leitura e escrita para a comunicação entre tarefas estão no arquivo <a href="https://github.com/carolinesma/ssd1306_rtos_i2c/blob/main/Core/Src/filas_rtos.c"> filas_rtos.c</a>.<br />As rotinas para criação dos semáforos estão no arquivo<a href="https://github.com/carolinesma/ssd1306_rtos_i2c/blob/main/Core/Src/semaforos.c"> semaforos.c</a>.<br />O arquivo<a href="https://github.com/carolinesma/ssd1306_rtos_i2c/blob/main/Core/Src/teste_escrita.c"> teste_escrita.c</a> é para realizar o teste das filas</p>

<p>Especificações:

* Placa: STM32F4<br />
* Display: SSD1306<br />
* Protocolo de Comunicação: I2C<br />
* Uso do FreeRtos</p>

A biblioteca ultilizada para o display é https://github.com/afiskon/stm32-ssd1306
