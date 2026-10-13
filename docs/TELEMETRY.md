# Telemetria MQTT com sinais reais

Configurar `include/secrets.h` e `include/network_config.h` a partir dos respectivos
cabeçalhos de exemplo. Também configurar `dashboard/config.py` a partir de
`dashboard/config.example.py`, usando o mesmo broker, porta e tópico.

O firmware usa Wi-Fi/MQTT por padrão. Para alternar o transporte, editar
`FEATURE_TELEMETRY_BACKEND` em `include/config/features.h`:

- `TELEMETRY_BACKEND_WIFI_MQTT` (padrão): publica o snapshot no tópico MQTT;
- `TELEMETRY_BACKEND_TELEPLOT`: envia as séries de diagnóstico pela serial.

Os backends são exclusivos. `FEATURE_TELEMETRY=0` desliga a tarefa de
telemetria e o código de rede; `FEATURE_TELEMETRY=1` mantém o backend escolhido.

```sh
pio run -e firmware -t upload
pio device monitor -e firmware
# Demo standalone, sem sensores ou driver de motor conectados:
pio run -e test_telemetry -t upload
pio device monitor -e test_telemetry
```

Iniciar o dashboard em um ambiente Python com as dependências de
`dashboard/requirements.txt` instaladas:

```sh
streamlit run dashboard/dashboard.py
```

## Campos publicados

O campo de topo `source` identifica a origem como `live` ou `synthetic`. Os grupos
são enviados na frequência `cfg::TELEMETRY_HZ` (atualmente 20 Hz):

| Grupo | Campos | Significado |
|---|---|---|
| `robot` | `mode`, `fault` | Estado corrente do supervisor e motivo de falha (`-` quando não há falha) |
| `health` | `line_available`, `imu_available`, `imu_status`, `control_available` | Presença de amostras recentes; a IMU também informa inicialização/calibração |
| `rates` | `control_hz`, `ir_sample_hz`, `imu_sample_hz` | Taxas efetivas medidas pelas tarefas, em Hz; zero quando a amostra não está disponível |
| `ir` | `raw`, `on_line` | Seis leituras ADC (0–4095) e seis indicadores de detecção da linha, na ordem dos pinos configurados |
| `ir` | `line_detected`, `position` | Posição de 0 a 5000; `null` quando a linha é perdida |
| `imu` | `accel_x_g`, `accel_y_g`, `accel_z_g` | Aceleração em g |
| `imu` | `gyro_z_dps` | Taxa angular de guinada em graus por segundo, com remoção de bias e filtro IIR do firmware |
| `imu` | `gyro_z_raw_dps` | Taxa angular do eixo Z com remoção de bias, antes do filtro do firmware |
| `motors` | `left_applied`, `right_applied` | Comandos normalizados com sinal, após ajuste de equilíbrio entre os lados (trim) e saturação, exibidos como porcentagens |
| `motors` | `armed` | Estado de habilitação do driver |

O grupo `imu` inteiro é enviado como `null` se a inicialização ou calibração falhar,
ou se nenhuma amostra do MPU chegar dentro de `cfg::IMU_TIMEOUT_MS`. Corrija
problemas de ligação ou do sinal de dados prontos (data-ready) e reinicie após uma
falha de inicialização. As leituras ADC dos sensores IR, por si só, não identificam
sensores desconectados.

`health.line_available` e `health.control_available` exigem uma amostra com até
três períodos de controle; a IMU usa `cfg::IMU_TIMEOUT_MS`. `line_detected`
separa uma leitura válida sem linha de uma falha de amostragem.

Os valores dos motores são comandos do driver, **não velocidade medida das rodas
nem ciclo de trabalho do PWM**. Eles permanecem em zero enquanto o driver estiver
desarmado. Ao integrar o controle, use a mesma instância de `MotorDriver` para
acionamento e coleta das amostras.

O firmware publica apenas leituras reais e não inclui erro de controle ou tempos
fictícios. O estado da conexão MQTT continua visível no dashboard,
independentemente das métricas dos sensores. Dados sintéticos existem somente no
ambiente de demonstração `test_telemetry`.

## Aquisição e acesso aos dispositivos

As tarefas em `tasks.cpp` possuem as instâncias da HAL: `TaskLineSensor` atualiza
o array IR, `TaskImu` consome as amostras sinalizadas pelo MPU e `TaskControl`
publica o estado do controlador e dos comandos aplicados. `TaskTelemetry` combina
os estados recentes e trata MQTT no núcleo 0.

`robotTelemetrySetup()` cria uma fila de profundidade 1 na tarefa de telemetria.
`robotTelemetrySetSample()` sobrescreve a fila sem esperar;
`robotTelemetryLoop()` consome a amostra mais recente. Amostras já consumidas não
são republicadas. As tentativas de conexão MQTT podem bloquear apenas essa tarefa,
sem bloquear a malha de controle.

`FEATURE_TELEMETRY=0` desabilita a rede no firmware; o ambiente específico de teste
de telemetria continua funcionando. A comunicação usa MQTT e o dashboard Python
existente. O servidor WebSocket embarcado permanece planejado para o futuro.

O ambiente `test_telemetry` publica dados sintéticos por 5 segundos e deixa a fila
sem amostra pelos próximos 5 segundos. No dashboard, isso demonstra a chegada dos
campos e o aviso de leituras antigas. Todos os valores do alvo são sintéticos;
ele não inicializa sensores nem motores. O firmware `firmware` publica os valores
reais obtidos pelas tarefas de sensores e controle.

## Verificação em bancada

- Conferir se os seis valores IR variam e se a posição da linha fica indisponível
  quando nenhum sensor detecta a linha.
- Manter o MPU imóvel durante a calibração. Depois, girá-lo para conferir os
  sinais da aceleração e das taxas angulares. A ausência do sinal de dados prontos
  deve fazer o MPU aparecer como indisponível.
- Confirmar que os comandos dos motores permanecem em zero com o driver desarmado.
- Parar e reiniciar o broker: a aquisição deve continuar, e as leituras atuais devem
  voltar a aparecer após a reconexão. Envios locais com QoS 0 não garantem entrega.
