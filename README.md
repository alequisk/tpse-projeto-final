### PROJETO FINAL TPSE: Automatização de luzes através do sensor de presença

O projeto final proposto para a disciplina de Técnicas para Sistemas Embarcados é criar uma aplicação embarcada na qual monitore o fluxo de pessoas em um cômodo, como um quarto de uma casa ou um auditório, e assim poderá fazer o acionamento automático de um circuito responsável pela iluminação do local.

## Ideia de usabilidade

Será aplicada a porta do lugar a ser monitorado a placa, juntamente com dois sensores de presença. A necessidade de ter dois ao invés de apenas um é obter também a informação de qual direção o indivíduo está.

## Ideia para o funcionamento

- Será feito um monitoramento por interrupção quando o sensor de presença for acionado;
- Após alguma pessoa entrar, o módulo de GPIO será responsável por acender ou apagar as luzes;
- Quando a ultima pessoa sair do local, o módulo timer será acionado para que seja dado um tempo antes que as luzes serem totalmente apagadas;
