# Telemetria MQTT com sinais reais

Configurar `include/secrets.h` e `include/network_config.h` a partir dos respectivos
cabeçalhos de exemplo. Também configurar `dashboard/config.py` a partir de
`dashboard/config.example.py`, usando o mesmo broker, porta e tópico.

```sh
pio run -e firmware -t upload
pio device monitor -e firmware
# Como alternativa, o teste de hardware usa o mesmo caminho de aquisição:
pio run -e test_telemetry -t upload
```

Iniciar o dashboard em um ambiente Python com as dependências de
`dashboard/requirements.txt` instaladas:

```sh
streamlit run dashboard/dashboard.py
```

## Campos publicados

Apenas estes três grupos são enviados, na frequência `cfg::TELEMETRY_HZ`
(atualmente 20 Hz):

| Grupo | Campos | Significado |
|---|---|---|
| `ir` | `raw`, `on_line` | Seis leituras ADC (0–4095) e seis indicadores de detecção da linha, na ordem dos pinos configurados |
| `ir` | `line_detected`, `position` | Posição de 0 a 5000; `null` quando a linha é perdida |
| `imu` | `accel_x_g`, `accel_y_g`, `accel_z_g` | Aceleração em g |
| `imu` | `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps` | Taxas angulares em graus por segundo, com remoção de bias; o eixo Z também passa pelo filtro IIR do firmware |
| `imu` | `gyro_z_raw_dps` | Taxa angular do eixo Z com remoção de bias, antes do filtro do firmware |
| `motors` | `left_applied`, `right_applied` | Comandos normalizados com sinal, após ajuste de equilíbrio entre os lados (trim) e saturação, exibidos como porcentagens |
| `motors` | `armed` | Estado de habilitação do driver |

O grupo `imu` inteiro é enviado como `null` se a inicialização ou calibração falhar,
ou se nenhuma amostra do MPU chegar dentro de `cfg::IMU_TIMEOUT_MS`. Corrija
problemas de ligação ou do sinal de dados prontos (data-ready) e reinicie após uma
falha de inicialização. As leituras ADC dos sensores IR, por si só, não identificam
sensores desconectados.

Os valores dos motores são comandos do driver, **não velocidade medida das rodas
nem ciclo de trabalho do PWM**. Eles permanecem em zero enquanto o driver estiver
desarmado. Ao integrar o controle, use a mesma instância de `MotorDriver` para
acionamento e coleta das amostras.

As leituras sintéticas, o erro de controle e os tempos fictícios, além dos antigos
campos de tempo de atividade, RSSI e conexão, foram removidos. O estado da conexão
MQTT continua visível no dashboard, independentemente das métricas dos sensores.

## Aquisição e acesso aos dispositivos

`robot_app.cpp` gerencia as instâncias da HAL. Seu laço principal amostra os
sensores IR na frequência configurada e consome as amostras sinalizadas pelo MPU,
copiando uma amostra conjunta na frequência da telemetria. Essas frequências são
obtidas por consulta periódica cooperativa, sem garantia de tempo real estrito.
O LED usa o tempo decorrido, sem bloquear a aquisição por um segundo.

`robotTelemetrySetup()` cria uma fila de profundidade 1 antes do início das
tarefas de aquisição e rede. `robotTelemetrySetSample()` sobrescreve a fila sem
esperar; `robotTelemetryLoop()` consome a amostra mais recente no núcleo 0.
Amostras já consumidas não são republicadas. As tentativas de conexão MQTT podem
bloquear apenas a tarefa de rede. A implementação completa das tarefas dedicadas
aos sensores e ao controle ainda está prevista para uma etapa futura.

`FEATURE_TELEMETRY=0` desabilita a rede no firmware; o ambiente específico de teste
de telemetria continua funcionando. A comunicação usa MQTT e o dashboard Python
existente. O servidor WebSocket embarcado permanece planejado para o futuro.

## Verificação em bancada

- Conferir se os seis valores IR variam e se a posição da linha fica indisponível
  quando nenhum sensor detecta a linha.
- Manter o MPU imóvel durante a calibração. Depois, girá-lo para conferir os
  sinais da aceleração e das taxas angulares. A ausência do sinal de dados prontos
  deve fazer o MPU aparecer como indisponível.
- Confirmar que os comandos dos motores permanecem em zero com o driver desarmado.
- Parar e reiniciar o broker: a aquisição deve continuar, e as leituras atuais devem
  voltar a aparecer após a reconexão. Envios locais com QoS 0 não garantem entrega.
