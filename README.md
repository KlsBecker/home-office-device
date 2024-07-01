# Home Office Device

## Descrição
O HomeOffice Device é um dispositivo baseado em ESP32 projetado para monitorar e controlar o ambiente do seu escritório em casa. Ele utiliza um sensor INA219 para monitorar tensão, corrente e potência, um display PCD8544 (Nokia 5110) para exibir informações e um relé para controlar dispositivos alimentados. A comunicação com o ESP32 é feita através de TCP, permitindo que os dados sejam lidos e que o estado do relé seja controlado por um dispositivo client.

## Uso
O HomeOffice Device foi projetado para ser utilizado com um dispositivo client TCP, que comunica com o ESP32 para ler os dados do sensor INA219 e controlar o estado do relé. O dispositivo também fornece feedback visual para o usuário através do display PCD8544 e permite que o estado do relé seja alterado através de um botão físico.

### Display PCD8544
O display mostra as leituras atuais do sensor INA219, incluindo tensão, corrente e potência, bem como o estado atual do relé.

### Botão
Um botão físico permite que o usuário mude o estado do relé manualmente, ligando ou desligando os dispositivos conectados.


#### Comunicação
A comunicação entre o cliente e o servidor é realizada via TCP, com frames fixos de 128 bytes, sendo o primeiro destinado ao comando e os demais aos dados.

#### Formatos das Requisições e Respostas:

1. **Formato da Requisição**:
   - **Estrutura do Frame**:
     - `cmd` [1 byte]: Código do comando
     - `data` [127 bytes]: Não utilizado na requisição, pode ser preenchido com zeros

2. **Formato da Resposta**:
   - **Estrutura do Frame**:
     - `cmd` [1 byte]: Código do comando recebido ou `CMD_UNKNOWN` para comando não reconhecido
     - `data` [127 bytes]: Dados requisitados ou preenchido com zeros no caso de erro

#### Comandos e Formatos das Respostas:

1. **Comando para Leitura da Tensão**:
   - **Código do Comando**: `0x01` (`CMD_READ_VOLTAGE`)
   - **Formato da Requisição**:
     ```
     | 0x01 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x01 | <float: 4 bytes> | 0x00 | ... | 0x00 |
     ```

2. **Comando para Leitura da Corrente**:
   - **Código do Comando**: `0x02` (`CMD_READ_CURRENT`)
   - **Formato da Requisição**:
     ```
     | 0x02 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x02 | <float: 4 bytes> | 0x00 | ... | 0x00 |
     ```

3. **Comando para Leitura da Potência**:
   - **Código do Comando**: `0x03` (`CMD_READ_POWER`)
   - **Formato da Requisição**:
     ```
     | 0x03 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x03 | <float: 4 bytes> | 0x00 | ... | 0x00 |
     ```

4. **Comando para Leitura do Estado do Relé**:
   - **Código do Comando**: `0x04` (`CMD_READ_RELAY`)
   - **Formato da Requisição**:
     ```
     | 0x04 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x04 | <uint8_t: 1 byte> | 0x00 | ... | 0x00 |
     ```

5. **Comando para Leitura de Todos os Valores**:
   - **Código do Comando**: `0x05` (`CMD_READ_ALL`)
   - **Formato da Requisição**:
     ```
     | 0x05 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x05 | <voltage float: 4 bytes> | <current float: 4 bytes> | <power float: 4 bytes> | <relay uint8_t: 1 byte> | 0x00 | ... | 0x00 |
     ```

6. **Comando para Ligar o Relé**:
   - **Código do Comando**: `0x06` (`CMD_SET_RELAY_ON`)
   - **Formato da Requisição**:
     ```
     | 0x06 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x06 | <uint8_t: 1 byte> | 0x00 | ... | 0x00 |
     ```

7. **Comando para Desligar o Relé**:
   - **Código do Comando**: `0x07` (`CMD_SET_RELAY_OFF`)
   - **Formato da Requisição**:
     ```
     | 0x07 | 0x00 | ... | 0x00 |
     ```
   - **Formato da Resposta**:
     ```
     | 0x07 | <uint8_t: 1 byte> | 0x00 | ... | 0x00 |
     ```

8. **Mensagem de Erro**:
   - **Código do Comando**: `0xFF` (`CMD_UNKNOWN`)
   - **Formato da Resposta**:
     ```
     | 0xFF | 0x00 | ... | 0x00 |
     ```

### Comandos Aceitos:

#### Requisições:
- **Tensão**: `CMD_READ_VOLTAGE (0x01)`
- **Corrente**: `CMD_READ_CURRENT (0x02)`
- **Potência**: `CMD_READ_POWER (0x03)`
- **Estado do Relé**: `CMD_READ_RELAY (0x04)`
- **Leitura de Todos os Valores**: `CMD_READ_ALL (0x05)`
- **Ligar Relé**: `CMD_SET_RELAY_ON (0x06)`
- **Desligar Relé**: `CMD_SET_RELAY_OFF (0x07)`

#### Respostas:
- **Tensão**: `CMD_READ_VOLTAGE (0x01) | <float>`
- **Corrente**: `CMD_READ_CURRENT (0x02) | <float>`
- **Potência**: `CMD_READ_POWER (0x03) | <float>`
- **Estado do Relé**: `CMD_READ_RELAY (0x04) | <uint8_t>`
- **Leitura de Todos os Valores**: `CMD_READ_ALL (0x05) | <voltage float> | <current float> | <power float> | <relay uint8_t>`
- **Ligar Relé**: `CMD_SET_RELAY_ON (0x06) | <uint8_t>`
- **Desligar Relé**: `CMD_SET_RELAY_OFF (0x07) | <uint8_t>`
- **Erro**: `CMD_UNKNOWN (0xFF)`

## Créditos
Este projeto utiliza as seguintes bibliotecas e recursos:
- [INA219 Driver](https://github.com/UncleRus/esp-idf-lib/tree/master/components/ina219): Um driver para o sensor INA219 para ESP-IDF.
- [PCD8544 Driver](https://github.com/yanbe/esp32-pcd8544): Uma biblioteca para controlar o display PCD8544 com ESP32 usando o ESP-IDF.
