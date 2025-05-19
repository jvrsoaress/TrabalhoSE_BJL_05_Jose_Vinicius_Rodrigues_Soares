<img width=100% src="https://capsule-render.vercel.app/api?type=waving&color=02A6F4&height=120&section=header"/>
<h1 align="center">Embarcatech - Projeto Integrado - BitDogLab </h1>

## Objetivo do Projeto

Um sistema de alerta de enchente utilizando o Raspberry Pi Pico W na plaquinha BitDogLab, permitindo o monitoramento simulado de nível de água e volume de chuva via joystick. O sistema exibe informações em tempo real no display OLED, sinaliza estados de alerta com a matriz de LEDs, LED RGB e buzzer, e alterna entre modos de operação (Nível de Água e Volume de Chuva), simulando um sistema de prevenção de enchentes.

## 🗒️ Lista de requisitos

- **Leitura de botões (A, B e Joystick):** Botão A (alterna modos), Botão B (entra no modo BOOTSEL), Joystick (simula nível de água e volume de chuva);
- **Utilização da matriz de LEDs:** Exibe o nível de água ou volume de chuva em azul, preenchendo de baixo para cima;
- **Utilização de LED RGB:** Sinaliza estados de alerta (verde para Seguro, amarelo para Alerta, vermelho para Emergência);
- **Display OLED (SSD1306):** Exibe modo atual, valor lido e estado de alerta com borda retangular;
- **Utilização do buzzer:** Emite som intermitente (200ms ligado, 800ms desligado) no estado Alerta e som contínuo no estado Emergência;
- **Estruturação do projeto:** Código em C no VS Code, usando Pico SDK e FreeRTOS, com comentários detalhados;
- **Técnicas implementadas:** ADC, I2C, PIO, PWM, interrupções, FreeRTOS com filas, e debounce via hardware;
  

## 🛠 Tecnologias

1. **Microcontrolador:** Raspberry Pi Pico W.
2. **Display OLED SSD1306:** 128x64 pixels, conectado via I2C (GPIO 14 - SDA, GPIO 15 - SCL).
3. **Joystick:** GPIOs 26 (eixo X, volume de chuva) e 27 (eixo Y, nível de água).
4. **Botão A:** GPIO 5 (alterna modos).
5. **Botão B:** GPIO 6 (entra no modo BOOTSEL).
6. **Matriz de LEDs:** WS2812 (GPIO 7).
7. **LED RGB:** GPIOs 11 (verde), 12 (azul), 13 (vermelho).
8. **Buzzer:** GPIO 10.
9. **Sensor de temperatura:** ADC canal 4 (sensor interno do RP2040).
10. **Linguagem de Programação:** C.
11. **Frameworks:** Pico SDK, FreeRTOS.


## 🔧 Funcionalidades Implementadas:

**Funções dos Componentes**

- **Matriz de LEDs (WS2812):** Mostra o nível de água ou volume de chuva em azul, preenchendo de baixo para cima proporcionalmente ao valor lido (0 a 4095).
- **LED RGB:** Sinaliza o estado de alerta: verde (Seguro), amarelo (Alerta), vermelho (Emergência). 
- **Display OLED:** Exibe em tempo real:
  - Modo atual ("Nível Água" ou "Volume Chuva").
  - Valor lido do joystick (0 a 4095).
  - Estado de alerta ("Seguro", "Alerta", "Emergência").
- **Buzzer:** Emite som intermitente no estado Alerta e som contínuo no estado Emergência.
- **Joystick:** Simula nível de água (eixo Y) e volume de chuva (eixo X).
- **Botões:** 
  - Botão A: Alterna entre os modos Nível de Água e Volume de Chuva com debounce de 200ms.
  - Botão B: Reinicia a placa no modo BOOTSEL para atualizações via USB.
- **Técnicas:**
  - Uso de filas FreeRTOS para comunicação entre tarefas.
  - Leitura do joystick a cada 100ms via ADC.
  - Interrupções para os botões com debounce de 200ms.
  - I2C para OLED, PIO para matriz WS2812, PWM para buzzer.

## 🚀 Passos para Compilação e Upload do projeto Ohmímetro com Matriz de LEDs

1. **Instale o Pico SDK:** Configure o ambiente com Pico SDK e bibliotecas lwIP.
2. **Crie uma pasta `build`**:
   ```bash
   mkdir build
   cd build
   cmake ..
   make

3. **Transferir o firmware para a placa:**

- Conectar a placa BitDogLab ao computador via USB.
- Copiar o arquivo .uf2 gerado para o drive da placa.

4. **Testar o projeto**

- Após o upload, desconecte e reconecte a placa.
- Use o joystick para simular nível de água e volume de chuva.
- Pressione o botão A para alternar modos e o botão B para entrar no modo BOOTSEL.

🛠🔧🛠🔧🛠🔧


## 🎥 Demonstração: 

- Para ver o funcionamento do projeto, acesse o vídeo de demonstração gravado por José Vinicius em: https://youtu.be/qyIR7HiaAmI

## 💻 Desenvolvedor
 
<table>
  <tr>
    <td align="center"><img style="" src="https://avatars.githubusercontent.com/u/191687774?v=4" width="100px;" alt=""/><br /><sub><b> José Vinicius </b></sub></a><br />👨‍💻</a></td>
  </tr>
</table>
