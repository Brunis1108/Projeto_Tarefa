# 🚦 Semáforo Inteligente com Modo Noturno e Acessibilidade Sonora

Projeto desenvolvido utilizando a placa **BitDogLab** com o microcontrolador **RP2040**, integrando controle visual via matriz de LEDs WS2812 e acessibilidade sonora por meio de um buzzer, com alternância entre **modo normal** e **modo noturno**.

---

## 📋 Descrição

Este projeto simula um semáforo com foco em acessibilidade para pessoas com deficiência visual. Utiliza o sistema **FreeRTOS** para gerenciar múltiplas tarefas paralelas de forma eficiente.

- **Modo Normal:** Semáforo tradicional com as cores **vermelho**, **verde** e **amarelo**.
- **Modo Noturno:** Amarelo piscando a cada segundo.
- **Acessibilidade:** Buzzer emite sons distintos conforme a cor do semáforo para guiar pessoas cegas.

---

## 🛠️ Funcionalidades

- ✅ Controle de LED RGB com WS2812 (formando o semáforo).
- ✅ Alternância de modo com botão físico (botão A).
- ✅ Reinicialização segura via botão B (modo BOOTSEL).
- ✅ Sinalização sonora com frequências diferentes para cada cor:
  - 🔴 Vermelho: 440 Hz a cada 1s (alerta para esperar)
  - 🟢 Verde: 800 Hz a cada 100ms (sinal para atravessar)
  - 🟡 Amarelo: 500 Hz a cada 200ms (atenção)
- ✅ FreeRTOS para organização das tarefas.

---

## ⚙️ Hardware Utilizado

- 📦 **BitDogLab com RP2040**
- 🔲 **Matriz de LEDs WS2812** (5x5) - GPIO 7
- 🔘 **Botão A** (modo noturno) - GPIO 5  
- 🔘 **Botão B** (reset BOOTSEL) - GPIO 6  
- 🔊 **Buzzer** - GPIO 10  
- 🧠 **FreeRTOS** - para controle de tarefas

---
## Link Video
<https://www.youtube.com/watch?v=YMIQkdILDB4>
  
