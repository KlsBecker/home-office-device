# Home Office Device

## Descrição
O HomeOffice Device é um dispositivo baseado em ESP32 projetado para monitorar e controlar o ambiente do seu escritório em casa. Ele utiliza um sensor INA219 para monitorar tensão, corrente e potência, um display PCD8544 (Nokia 5110) para exibir informações e um relé para controlar dispositivos alimentados. A comunicação com o ESP32 é feita através de SPI, permitindo que os dados sejam lidos e que o estado do relé seja controlado por um dispositivo mestre.

## Requisitos

### Requisitos Funcionais
1. **RF01 - Leitura de Dados do INA219:**
   - O sistema deve ler dados de tensão, corrente e potência do sensor INA219 via I2C.
   
2. **RF02 - Exibição de Dados no Display:**
   - Os dados do INA219 e o estado do relé devem ser exibidos no display PCD8544.

3. **RF03 - Controle do Relé:**
   - O sistema deve ser capaz de ligar e desligar um relé usando um pino GPIO do ESP32.

4. **RF04 - Comunicação SPI:**
   - O ESP32 deve comunicar via SPI com outro dispositivo, permitindo a transmissão dos dados do INA219 e recebendo comandos para controlar o relé.

5. **RF05 - Atualização do Display:**
   - O display PCD8544 deve ser atualizado em intervalos regulares para mostrar os dados mais recentes.

### Requisitos Não Funcionais
1. **RNF01 - Precisão nas Leituras:**
   - As leituras do INA219 devem ser precisas e confiáveis.

2. **RNF02 - Taxa de Atualização do Display:**
   - O display PCD8544 deve ser atualizado com uma frequência que permita uma leitura confortável e que represente de forma fidedigna as alterações nos dados medidos.

3. **RNF03 - Resiliência na Comunicação SPI:**
   - O sistema deve ser capaz de lidar com interrupções ou falhas na comunicação SPI, garantindo uma retomada suave da comunicação.

4. **RNF04 - Alimentação:**
   - O sistema deve operar de maneira estável e confiável a partir de uma alimentação via USB.

5. **RNF05 - Segurança na Operação do Relé:**
   - O controle do relé deve ser realizado de maneira segura, evitando operações indesejadas que possam causar danos aos dispositivos conectados.

6. **RNF06 - Usabilidade:**
   - Os dados no display devem ser facilmente legíveis e a interação via SPI deve ser intuitiva e fácil de usar.

## Diagrama de Blocos
![Alt](HomeOfficeDevice.drawio.svg)

## Uso
O HomeOffice Device foi projetado para ser utilizado com um dispositivo mestre SPI, que comunica com o ESP32 para ler os dados do sensor INA219 e controlar o estado do relé. O dispositivo também fornece feedback visual para o usuário através do display PCD8544 e permite que o estado do relé seja alterado através de um botão físico.

### Display PCD8544
O display mostra as leituras atuais do sensor INA219, incluindo tensão, corrente e potência, bem como o estado atual do relé.

### Botão
Um botão físico permite que o usuário mude o estado do relé manualmente, ligando ou desligando os dispositivos conectados.

## Créditos
Este projeto utiliza as seguintes bibliotecas e recursos:
- [INA219 Driver](https://github.com/UncleRus/esp-idf-lib/tree/master/components/ina219): Um driver para o sensor INA219 para ESP-IDF.
- [PCD8544 Driver](https://github.com/yanbe/esp32-pcd8544): Uma biblioteca para controlar o display PCD8544 com ESP32 usando o ESP-IDF.
