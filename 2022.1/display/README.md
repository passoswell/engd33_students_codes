# Aplicação de monitoramento de variáveis em display com FreeRTOS

A seguinte aplicação foi desenvolvida no âmbito da disciplina Programação em Tempo Real para Sistemas Embarcados, cursada na Universidade Federal da Bahia (UFBA) no semestre 2022.1.

A aplicação é uma prova de conceito em um cenário de testes simulando a monitoração de variáveis de posicionamento, velocidade e consumo de corrente no sistema eletrônico de um robô.

## Tipos de task criadas

Foram criados os seguintes tipos de tasks:

*   Tasks para gerenciamento do display;
*   Tasks para gerenciamento de interrupções requisitadas pelos botões (para comutação das informações do display);
*   Tasks para geração de dados para o cenário de simulação;
*   Tasks para leitura dos dados gerados via queue e gravação em buffer circular.

Os dados gerados não necessariamente condizem com valores esperados em um cenário real.