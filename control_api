import pygame
import sim
import time
import matplotlib.pyplot as plt
import math
import statistics
import os


# ============================================================
# FUNÇÕES AUXILIARES
# ============================================================

# Função logística utilizada para suavizar a resposta
# do joystick e reduzir sensibilidade próxima do zero
def curva_logistica(x, k=3):
    return (2 / (1 + math.exp(-k * x))) - 1


# Limita um valor entre mínimo e máximo
def clamp(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)


# Atualiza a velocidade considerando limite máximo
# de aceleração para evitar mudanças abruptas
def update_vel(vel_atual, vel_des, acc_max, dt):
    delta = vel_des - vel_atual
    delta = clamp(delta, -acc_max * dt, acc_max * dt)
    return vel_atual + delta


# função genérica para gerar gráficos
def gerar_grafico(
    x,
    ys,
    labels,
    xlabel,
    ylabel,
    titulo,
    nome_arquivo
):

    plt.figure(figsize=(10, 5))

    for y, label in zip(ys, labels):
        plt.plot(x, y, label=label)

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    plt.title(titulo)

    if len(labels) > 1:
        plt.legend()

    plt.grid()

    plt.tight_layout()

    # Salva o gráfico
    plt.savefig(
        os.path.join(
            pasta_graficos,
            nome_arquivo
        ),
        dpi=300,
        bbox_inches='tight'
    )

    # Mostra na tela
    plt.show()

    # Fecha a figura
    plt.close()


# ============================================================
# CONFIGURAÇÕES DO SISTEMA
# ============================================================
# Ajuste os valores abaixo para calibrar o comportamento do
# rover e do braço robótico. Nenhum outro trecho do código
# precisa ser alterado para fins de calibração.
# ============================================================

# --------------------------------------------------------
# Chaves gerais (liga/desliga comportamentos)
# --------------------------------------------------------

# Ativa/desativa suavização logística do joystick
usar_suavizacao = True

# Ativa/desativa limitação de aceleração
usar_limite_aceleracao = True

# Ativa/desativa limitação de velocidade máxima
usar_limite_velocidade = True

# --------------------------------------------------------
# Suavização do joystick (curva logística)
# Quanto maior o K, mais "agressiva"/rápida é a resposta
# perto do centro do manche; quanto menor, mais suave.
# --------------------------------------------------------

K_SUAVIZACAO_ROVER = 0.2   # usado nos eixos de velocidade e direção do rover
K_SUAVIZACAO_BRACO = 0.5   # usado nos eixos do braço robótico (X, Y, Y2)

# --------------------------------------------------------
# Deadzone dos eixos do joystick
# --------------------------------------------------------

DEADZONE_ROVER = 0.1
DEADZONE_BRACO = 0.1

# --------------------------------------------------------
# Velocidade máxima e aceleração do ROVER (rodas)
# --------------------------------------------------------

VEL_MAX_ROVER = 15          # velocidade máxima quando o limite está ativo
VEL_MAX_ROVER_SEM_LIMITE = 100  # velocidade máxima quando o limite está desativado

ACELERACAO_MAX_ROVER = 5    # aceleração máxima (unid./s²)
DESACELERACAO_MAX_ROVER = 10  # desaceleração máxima (unid./s²)

# --------------------------------------------------------
# Direção (steering) do ROVER
# --------------------------------------------------------

VEL_STEER_MAX = 3.0         # velocidade angular máxima de esterçamento
ACC_STEER_BASE = 5.0        # aceleração angular base de esterçamento

# Ganhos do retorno automático do volante ao centro (PD)
KP_STEER_CENTRO = 25.0
KD_STEER_CENTRO = 8.0

MAX_STEER_ANGLE = 0.5       # limite físico do ângulo de esterçamento (rad)

# --------------------------------------------------------
# Velocidade e aceleração do BRAÇO ROBÓTICO
# --------------------------------------------------------

VEL_MAX_BRACO = 1.5         # velocidade máxima das juntas do braço
ACC_MAX_BRACO = 4.0         # aceleração máxima das juntas do braço

# --------------------------------------------------------
# Limites físicos (curso) das juntas do braço robótico
# --------------------------------------------------------

MAX_MANI_P1 = 1.0           # junta P1: vai de -MAX_MANI_P1 a +MAX_MANI_P1

MAX_MANI_P2 = 1.05          # junta P2: vai de -MAX_MANI_P2 a +MAX_MANI_P2

MIN_MANI_P3 = -0.52         # junta P3: curso mínimo
MAX_MANI_P3 = 1.92          # junta P3: curso máximo

MAX_MANI_TOOL = 1.92        # ferramenta: curso máximo (mínimo é 0)


# ============================================================
# VARIÁVEIS DE LOG E MONITORAMENTO
# ============================================================

# Vetores utilizados para armazenamento de dados
# da simulação para análise posterior e geração de gráficos
tempo = []

vel_log = []
input_vel_log = []

angulo_steer_log = []
input_steer_log = []

# Logs relacionados ao tempo de cena do CoppeliaSim
# (substituem a antiga medição de FPS baseada no
# relógio local do processo Python)
fps_cena_log = []
fps_cena_tempo = []

# Logs do FPS medido diretamente dentro do CoppeliaSim
# via script Lua (sinal "fps_coppelia")
fps_lua_log = []
fps_lua_tempo = []

# Logs SINCRONIZADOS (mesmo eixo de tempo a cada iteração do
# loop) usados exclusivamente para a comparação direta entre
# as duas fontes de FPS no mesmo gráfico
fps_compare_tempo = []
fps_compare_cena = []
fps_compare_lua = []

ultimo_fps_cena = 0.0
ultimo_fps_lua = 0.0

tempo_cena_log = []
tempo_real_log = []

tempo_resposta = []

pos_p1_log = []
pos_p2_log = []
pos_p3_log = []
pos_tool_log = []

input_x_log = []
input_y_log = []
input_y2_log = []
input_rt_log = []


# ============================================================
# CONTROLE DE TEMPO
# ============================================================

# perf_counter possui maior precisão para medições
# temporais e benchmark do sistema
start_time = time.perf_counter()
last_time = start_time


# ============================================================
# CONEXÃO COM O COPPELIASIM
# ============================================================

sim.simxFinish(-1)

clientID = sim.simxStart(
    '127.0.0.1',
    19997,
    True,
    True,
    5000,
    5
)

if clientID != -1:
    print("Conectado ao CoppeliaSim")
else:
    print("Falha na conexão com o CoppeliaSim")
    exit()


# ============================================================
# INICIALIZAÇÃO DO JOYSTICK
# ============================================================

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controle conectado:", joystick.get_name())


# ============================================================
# CONTROLE DE MODOS DE OPERAÇÃO
# False = movimentação do rover
# True  = controle do braço robótico
# ============================================================

modo_braco = False
start_anterior = False


# ============================================================
# HANDLES DAS RODAS
# ============================================================

_, joint_wheel_front_right = sim.simxGetObjectHandle(
    clientID,
    'joint_wheel_front_right',
    sim.simx_opmode_blocking
)

_, joint_wheel_mid_right = sim.simxGetObjectHandle(
    clientID,
    'joint_wheel_mid_right',
    sim.simx_opmode_blocking
)

_, joint_wheel_back_right = sim.simxGetObjectHandle(
    clientID,
    'joint_wheel_back_right',
    sim.simx_opmode_blocking
)

_, joint_wheel_front_left = sim.simxGetObjectHandle(
    clientID,
    'joint_wheel_front_left',
    sim.simx_opmode_blocking
)

_, joint_wheel_mid_left = sim.simxGetObjectHandle(
    clientID,
    'joint_wheel_mid_left',
    sim.simx_opmode_blocking
)

_, joint_wheel_back_left = sim.simxGetObjectHandle(
    clientID,
    'joint_wheel_back_left',
    sim.simx_opmode_blocking
)


# ============================================================
# HANDLES DAS JUNTAS DE DIREÇÃO
# ============================================================

_, joint_support_front_left = sim.simxGetObjectHandle(
    clientID,
    'joint_support_front_left',
    sim.simx_opmode_blocking
)

_, joint_support_front_right = sim.simxGetObjectHandle(
    clientID,
    'joint_support_front_right',
    sim.simx_opmode_blocking
)

_, joint_support_back_left = sim.simxGetObjectHandle(
    clientID,
    'joint_support_back_left',
    sim.simx_opmode_blocking
)

_, joint_support_back_right = sim.simxGetObjectHandle(
    clientID,
    'joint_support_back_right',
    sim.simx_opmode_blocking
)


# ============================================================
# HANDLES DAS JUNTAS DO BRAÇO ROBÓTICO
# ============================================================

_, joint_mani_p1 = sim.simxGetObjectHandle(
    clientID,
    'joint_mani_p1',
    sim.simx_opmode_blocking
)

_, joint_mani_p2 = sim.simxGetObjectHandle(
    clientID,
    'joint_mani_p2',
    sim.simx_opmode_blocking
)

_, joint_mani_p3 = sim.simxGetObjectHandle(
    clientID,
    'joint_mani_p3',
    sim.simx_opmode_blocking
)

_, joint_mani_tool = sim.simxGetObjectHandle(
    clientID,
    'joint_mani_tool',
    sim.simx_opmode_blocking
)


# ============================================================
# INICIALIZAÇÃO DO STREAMING DA VELOCIDADE REAL
# ============================================================

# O parâmetro 2012 corresponde à velocidade angular
# da junta no CoppeliaSim
sim.simxGetObjectFloatParameter(
    clientID,
    joint_wheel_front_left,
    2012,
    sim.simx_opmode_streaming
)


# ============================================================
# AGRUPAMENTO DOS MOTORES
# ============================================================

motores_esq = [
    joint_wheel_front_left,
    joint_wheel_mid_left,
    joint_wheel_back_left
]

motores_dir = [
    joint_wheel_front_right,
    joint_wheel_mid_right,
    joint_wheel_back_right
]

fatores_esq = [1, 1, 1]
fatores_dir = [1, 1, 1]


# ============================================================
# ESTADOS DO SISTEMA
# ============================================================

vel_wheel = 0

vel_p1 = 0
vel_p2 = 0
vel_p3 = 0
vel_tool = 0

angulo_steer = 0
vel_steer = 0
eixo_dir = 0

pos_mani_p1 = 0
pos_mani_p2 = 0
pos_mani_p3 = 0
pos_mani_tool = 0

contador_print = 0


# ============================================================
# CONTROLE DE MEDIÇÃO DE LATÊNCIA
# ============================================================

tempo_input = None
input_detectado = False


# ============================================================
# REFERÊNCIA INICIAL DO TEMPO DE CENA (COPPELIASIM)
# ============================================================
# simxGetLastCmdTime retorna o tempo de simulação (em ms)
# associado ao último comando processado pelo servidor.
# Essa métrica é usada como base para o cálculo do FPS de
# cena e para a comparação entre tempo de cena e tempo real.
# ============================================================

tempo_cena_anterior = sim.simxGetLastCmdTime(clientID) / 1000.0

# ============================================================
# INICIALIZAÇÃO DO STREAMING DO SINAL DE FPS (SCRIPT LUA)
# ============================================================
# O sinal "fps_coppelia" é publicado pelo Child Script Lua
# anexado à cena (ver medidor_fps.lua). A primeira chamada
# em modo streaming inicia a leitura contínua do sinal.
# ============================================================

sim.simxGetFloatSignal(
    clientID,
    "fps_coppelia",
    sim.simx_opmode_streaming
)

# ============================================================
# LOOP PRINCIPAL
# ============================================================

rodando = True

while rodando:

    pygame.event.pump()

    # --------------------------------------------------------
    # Cálculo do delta de tempo do loop
    # --------------------------------------------------------

    current_time = time.perf_counter()

    dt = current_time - last_time
    last_time = current_time

    # --------------------------------------------------------
    # Cálculo do FPS a partir do tempo de cena do CoppeliaSim
    # --------------------------------------------------------
    # Em vez de medir o tempo decorrido no processo Python
    # (que inclui overhead de rede e processamento da API),
    # o FPS é derivado da variação do tempo de simulação
    # reportado diretamente pelo servidor do CoppeliaSim.
    # --------------------------------------------------------

    tempo_cena_atual = sim.simxGetLastCmdTime(clientID) / 1000.0
    delta_cena = tempo_cena_atual - tempo_cena_anterior

    if delta_cena > 0.001:

        fps_cena = 1 / delta_cena

        fps_cena_log.append(fps_cena)
        fps_cena_tempo.append(current_time - start_time)

        ultimo_fps_cena = fps_cena

    tempo_cena_log.append(tempo_cena_atual)
    tempo_real_log.append(current_time - start_time)

    tempo_cena_anterior = tempo_cena_atual

    # --------------------------------------------------------
    # Leitura do FPS medido dentro do CoppeliaSim (script Lua)
    # --------------------------------------------------------
    # Mede o tempo real (wall-clock) entre passos de atuação,
    # publicado como sinal pelo Child Script anexado à cena.
    # --------------------------------------------------------

    res_fps_lua, fps_lua = sim.simxGetFloatSignal(
        clientID,
        "fps_coppelia",
        sim.simx_opmode_buffer
    )

    if res_fps_lua == sim.simx_return_ok:

        fps_lua_log.append(fps_lua)
        fps_lua_tempo.append(current_time - start_time)

        ultimo_fps_lua = fps_lua

    # --------------------------------------------------------
    # Registro sincronizado (mesmo instante de tempo) das duas
    # fontes de FPS, para permitir a comparação direta no
    # mesmo gráfico
    # --------------------------------------------------------

    fps_compare_tempo.append(current_time - start_time)
    fps_compare_cena.append(ultimo_fps_cena)
    fps_compare_lua.append(ultimo_fps_lua)

    # --------------------------------------------------------
    # Encerrar simulação
    # --------------------------------------------------------

    if joystick.get_button(6):

        print("Encerrando simulação...")
        rodando = False

    # --------------------------------------------------------
    # Alternância entre rover e braço robótico
    # --------------------------------------------------------

    start_atual = joystick.get_button(7)

    if start_atual and not start_anterior:

        modo_braco = not modo_braco

        print("Modo braço:", modo_braco)

    start_anterior = start_atual

    # ========================================================
    # MODO ROVER
    # ========================================================

    if not modo_braco:

        eixo_vel = -joystick.get_axis(1)
        eixo_dir = joystick.get_axis(2)

        # ----------------------------------------------------
        # Deadzone do joystick
        # ----------------------------------------------------

        deadzone = DEADZONE_ROVER

        eixo_vel = 0 if abs(eixo_vel) < deadzone else eixo_vel
        eixo_dir = 0 if abs(eixo_dir) < deadzone else eixo_dir

        # ----------------------------------------------------
        # Suavização da entrada do joystick
        # ----------------------------------------------------

        if usar_suavizacao:

            eixo_vel = curva_logistica(eixo_vel, k=K_SUAVIZACAO_ROVER)
            eixo_dir = curva_logistica(eixo_dir, k=K_SUAVIZACAO_ROVER)

        # ----------------------------------------------------
        # Detecta o instante do comando do usuário
        # ----------------------------------------------------

        if abs(eixo_vel) > 0.2 and not input_detectado:

            tempo_input = current_time
            input_detectado = True

        # ====================================================
        # CONTROLE DE VELOCIDADE
        # ====================================================

        if usar_limite_velocidade:
            max_vel = VEL_MAX_ROVER
        else:
            max_vel = VEL_MAX_ROVER_SEM_LIMITE

        vel_desejada = eixo_vel * max_vel

        aceleracao_max = ACELERACAO_MAX_ROVER
        desaceleracao = DESACELERACAO_MAX_ROVER

        if usar_limite_aceleracao:

            delta_vel = vel_desejada - vel_wheel

            if abs(vel_desejada) > abs(vel_wheel):
                limite = aceleracao_max * dt
            else:
                limite = desaceleracao * dt

            delta_vel = clamp(delta_vel, -limite, limite)

            vel_wheel += delta_vel

        else:

            # Resposta instantânea sem limitação física
            vel_wheel = vel_desejada

        # ====================================================
        # MEDIÇÃO DE TEMPO DE RESPOSTA REAL
        # ====================================================

        erro, vel_real = sim.simxGetObjectFloatParameter(
            clientID,
            joint_wheel_front_left,
            2012,
            sim.simx_opmode_buffer
        )

        # Mede o atraso entre input e movimento real da roda
        if (
            input_detectado
            and erro == sim.simx_return_ok
            and (current_time - tempo_input) > 0.005
            and abs(vel_real) > 0.05
        ):

            delay = current_time - tempo_input

            tempo_resposta.append(delay)

            contador_print += 1

            if contador_print % 20 == 0:
                print(f"Tempo médio parcial: {delay:.6f} s")

            input_detectado = False

        # ====================================================
        # CONTROLE DE DIREÇÃO
        # ====================================================

        vel_steer_max = VEL_STEER_MAX
        acc_steer_base = ACC_STEER_BASE

        # Reduz sensibilidade em altas velocidades
        fator_vel = max(0.2, 1 - abs(vel_wheel) / max_vel)

        acc_steer_max = acc_steer_base * fator_vel
        vel_steer_lim = vel_steer_max * fator_vel

        if abs(eixo_dir) > 0.05:

            vel_steer_des = -eixo_dir * vel_steer_lim

            delta = vel_steer_des - vel_steer

            delta = clamp(
                delta,
                -acc_steer_max * dt,
                acc_steer_max * dt
            )

            vel_steer += delta

        else:

            # Retorno automático para posição central
            Kp = KP_STEER_CENTRO
            Kd = KD_STEER_CENTRO

            acc = (-Kp * angulo_steer - Kd * vel_steer)

            vel_steer += acc * dt

        # Integração do ângulo de direção
        angulo_steer += vel_steer * dt

        # Limite físico de esterçamento
        max_steer_angle = MAX_STEER_ANGLE

        angulo_steer = clamp(
            angulo_steer,
            -max_steer_angle,
            max_steer_angle
        )

        # ====================================================
        # ENVIO DOS COMANDOS AO COPPELIASIM
        # ====================================================

        for motor, fator in zip(motores_esq, fatores_esq):

            sim.simxSetJointTargetVelocity(
                clientID,
                motor,
                vel_wheel * fator,
                sim.simx_opmode_streaming
            )

        for motor, fator in zip(motores_dir, fatores_dir):

            sim.simxSetJointTargetVelocity(
                clientID,
                motor,
                vel_wheel * fator,
                sim.simx_opmode_streaming
            )

        # Aplicação do ângulo de direção
        sim.simxSetJointTargetPosition(
            clientID,
            joint_support_front_left,
            angulo_steer,
            sim.simx_opmode_streaming
        )

        sim.simxSetJointTargetPosition(
            clientID,
            joint_support_front_right,
            angulo_steer,
            sim.simx_opmode_streaming
        )

        sim.simxSetJointTargetPosition(
            clientID,
            joint_support_back_left,
            -angulo_steer,
            sim.simx_opmode_streaming
        )

        sim.simxSetJointTargetPosition(
            clientID,
            joint_support_back_right,
            -angulo_steer,
            sim.simx_opmode_streaming
        )

    # ========================================================
    # MODO BRAÇO ROBÓTICO
    # ========================================================

    else:

        eixo_x = joystick.get_axis(0)
        eixo_y = -joystick.get_axis(1)
        eixo_y2 = -joystick.get_axis(3)

        rt = joystick.get_axis(5)
        lt = joystick.get_axis(4)

        # ----------------------------------------------------
        # Deadzone
        # ----------------------------------------------------

        deadzone = DEADZONE_BRACO

        eixo_x = 0 if abs(eixo_x) < deadzone else eixo_x
        eixo_y = 0 if abs(eixo_y) < deadzone else eixo_y
        eixo_y2 = 0 if abs(eixo_y2) < deadzone else eixo_y2

        rt = -1 if abs(rt) < deadzone else rt
        lt = -1 if abs(lt) < deadzone else lt

        # ----------------------------------------------------
        # Suavização logística
        # ----------------------------------------------------

        if usar_suavizacao:

            eixo_x = curva_logistica(eixo_x, k=K_SUAVIZACAO_BRACO)
            eixo_y = curva_logistica(eixo_y, k=K_SUAVIZACAO_BRACO)
            eixo_y2 = curva_logistica(eixo_y2, k=K_SUAVIZACAO_BRACO)

        rt = (rt + 1) / 2
        lt = (lt + 1) / 2

        # ----------------------------------------------------
        # Velocidade desejada das juntas
        # ----------------------------------------------------

        vel_max = VEL_MAX_BRACO

        vel_p1_des = -eixo_x * vel_max
        vel_p2_des = eixo_y * vel_max
        vel_p3_des = eixo_y2 * vel_max
        vel_tool_des = (rt - lt) * vel_max

        # ----------------------------------------------------
        # Limite de aceleração angular
        # ----------------------------------------------------

        acc_max = ACC_MAX_BRACO

        if usar_limite_aceleracao:

            vel_p1 = update_vel(vel_p1, vel_p1_des, acc_max, dt)
            vel_p2 = update_vel(vel_p2, vel_p2_des, acc_max, dt)
            vel_p3 = update_vel(vel_p3, vel_p3_des, acc_max, dt)
            vel_tool = update_vel(vel_tool, vel_tool_des, acc_max, dt)

        else:

            vel_p1 = vel_p1_des
            vel_p2 = vel_p2_des
            vel_p3 = vel_p3_des
            vel_tool = vel_tool_des

        # ----------------------------------------------------
        # Integração numérica da posição angular
        # ----------------------------------------------------

        pos_mani_p1 += vel_p1 * dt
        pos_mani_p2 += vel_p2 * dt
        pos_mani_p3 += vel_p3 * dt
        pos_mani_tool += vel_tool * dt

        # ----------------------------------------------------
        # Limites físicos das juntas
        # ----------------------------------------------------

        max_mani_p1 = MAX_MANI_P1
        max_mani_p2 = MAX_MANI_P2

        min_mani_p3 = MIN_MANI_P3
        max_mani_p3 = MAX_MANI_P3

        max_mani_tool = MAX_MANI_TOOL

        pos_mani_p1 = clamp(
            pos_mani_p1,
            -max_mani_p1,
            max_mani_p1
        )

        pos_mani_p2 = clamp(
            pos_mani_p2,
            -max_mani_p2,
            max_mani_p2
        )

        pos_mani_p3 = clamp(
            pos_mani_p3,
            min_mani_p3,
            max_mani_p3
        )

        pos_mani_tool = clamp(
            pos_mani_tool,
            0,
            max_mani_tool
        )

        # ----------------------------------------------------
        # Aplicação das posições no simulador
        # ----------------------------------------------------

        sim.simxSetJointTargetPosition(
            clientID,
            joint_mani_p1,
            pos_mani_p1,
            sim.simx_opmode_streaming
        )

        sim.simxSetJointTargetPosition(
            clientID,
            joint_mani_p2,
            pos_mani_p2,
            sim.simx_opmode_streaming
        )

        sim.simxSetJointTargetPosition(
            clientID,
            joint_mani_p3,
            pos_mani_p3,
            sim.simx_opmode_streaming
        )

        sim.simxSetJointTargetPosition(
            clientID,
            joint_mani_tool,
            pos_mani_tool,
            sim.simx_opmode_streaming
        )

    # ========================================================
    # REGISTRO DOS DADOS
    # ========================================================

    t = current_time - start_time

    tempo.append(t)

    vel_log.append(vel_wheel)

    input_vel_log.append(
        eixo_vel if not modo_braco else 0
    )

    angulo_steer_log.append(angulo_steer)

    input_steer_log.append(
        eixo_dir if not modo_braco else 0
    )

    pos_p1_log.append(pos_mani_p1)
    pos_p2_log.append(pos_mani_p2)
    pos_p3_log.append(pos_mani_p3)
    pos_tool_log.append(pos_mani_tool)

    input_x_log.append(eixo_x if modo_braco else 0)
    input_y_log.append(eixo_y if modo_braco else 0)
    input_y2_log.append(eixo_y2 if modo_braco else 0)
    input_rt_log.append(rt if modo_braco else 0)

    # Pequena pausa para reduzir uso excessivo da CPU
    time.sleep(0.001)


# ============================================================
# FINALIZAÇÃO
# ============================================================

pygame.quit()

sim.simxStopSimulation(
    clientID,
    sim.simx_opmode_blocking
)

sim.simxFinish(clientID)

print("Simulação finalizada.")


# ============================================================
# RESULTADOS ESTATÍSTICOS
# ============================================================

if fps_cena_log:

    fps_medio = sum(fps_cena_log) / len(fps_cena_log)

    print(f"FPS médio de cena (simxGetLastCmdTime): {fps_medio:.2f}")
    print(f"FPS mínimo de cena: {min(fps_cena_log):.2f}")
    print(f"FPS máximo de cena: {max(fps_cena_log):.2f}")

if fps_lua_log:

    fps_lua_medio = sum(fps_lua_log) / len(fps_lua_log)

    print(f"FPS médio medido via script Lua: {fps_lua_medio:.2f}")
    print(f"FPS mínimo (Lua): {min(fps_lua_log):.2f}")
    print(f"FPS máximo (Lua): {max(fps_lua_log):.2f}")

if tempo_cena_log and tempo_real_log and tempo_real_log[-1] > 0:

    fator_tempo_real = tempo_cena_log[-1] / tempo_real_log[-1]

    print(f"Tempo de cena total: {tempo_cena_log[-1]:.3f} s")
    print(f"Tempo real total: {tempo_real_log[-1]:.3f} s")
    print(f"Fator de tempo real (cena/real): {fator_tempo_real:.3f}")


if tempo_resposta:

    media = statistics.mean(tempo_resposta)
    desvio = statistics.stdev(tempo_resposta)

    print(f"Tempo médio: {media:.6f} s")
    print(f"Tempo mínimo: {min(tempo_resposta):.6f} s")
    print(f"Tempo máximo: {max(tempo_resposta):.6f} s")
    print(f"Desvio padrão: {desvio:.6f} s")    


# ============================================================
# PASTA PARA SALVAR OS GRÁFICOS
# ============================================================

pasta_graficos = "graficos_simulacao"

os.makedirs(pasta_graficos, exist_ok=True)


# ============================================================
# GERAÇÃO DOS GRÁFICOS
# ============================================================

# ============================================================
# VELOCIDADE DO ROVER
# ============================================================

gerar_grafico(
    tempo,
    [vel_log],
    ["Velocidade"],
    "Tempo (s)",
    "Velocidade",
    "Velocidade do Rover ao Longo do Tempo",
    "velocidade_rover.png"
)


# ============================================================
# INPUT VS VELOCIDADE
# ============================================================

gerar_grafico(
    tempo,
    [input_vel_log, vel_log],
    ["Input do Joystick", "Velocidade Aplicada"],
    "Tempo (s)",
    "Valor",
    "Input do Controle vs Velocidade do Rover",
    "input_vs_velocidade.png"
)


# ============================================================
# FPS DE CENA (COPPELIASIM)
# ============================================================

gerar_grafico(
    fps_cena_tempo,
    [fps_cena_log],
    ["FPS de Cena (simxGetLastCmdTime)"],
    "Tempo (s)",
    "FPS",
    "FPS de Cena (CoppeliaSim) ao Longo do Tempo",
    "fps_cena.png"
)


# ============================================================
# FPS MEDIDO VIA SCRIPT LUA (COMPARAÇÃO)
# ============================================================
# Compara, no mesmo eixo de tempo, as duas fontes de medição
# do FPS de cena: uma obtida externamente via
# simxGetLastCmdTime (tempo de simulação) e outra medida
# internamente pelo script Lua (tempo real entre passos de
# atuação), como forma de validação cruzada.
# ============================================================

if fps_lua_log:

    gerar_grafico(
        fps_compare_tempo,
        [fps_compare_cena, fps_compare_lua],
        [
            "FPS de Cena (simxGetLastCmdTime)",
            "FPS via Script Lua (sim.getSystemTime)"
        ],
        "Tempo (s)",
        "FPS",
        "Comparação: FPS de Cena vs FPS via Script Lua",
        "fps_cena_vs_lua.png"
    )


# ============================================================
# TEMPO DE CENA VS TEMPO REAL
# ============================================================
# A reta de referência (Tempo Real vs Tempo Real) representa
# o comportamento ideal de execução em tempo real (fator = 1).
# Quanto mais próxima a curva do Tempo de Cena estiver dessa
# reta, mais fiel ao tempo real é a execução da simulação.
# ============================================================

gerar_grafico(
    tempo_real_log,
    [tempo_cena_log, tempo_real_log],
    ["Tempo de Cena (CoppeliaSim)", "Tempo Real (referência)"],
    "Tempo Real (s)",
    "Tempo (s)",
    "Tempo de Cena vs Tempo Real",
    "tempo_cena_vs_tempo_real.png"
)


# ============================================================
# ÂNGULO DE DIREÇÃO
# ============================================================

gerar_grafico(
    tempo,
    [angulo_steer_log],
    ["Ângulo"],
    "Tempo (s)",
    "Ângulo (rad)",
    "Ângulo de Direção ao Longo do Tempo",
    "angulo_direcao.png"
)


# ============================================================
# INPUT DE DIREÇÃO VS ÂNGULO DE ESTERÇAMENTO
# ============================================================

gerar_grafico(
    tempo,
    [input_steer_log, angulo_steer_log],
    ["Input do Controle (Direção)", "Ângulo de Esterçamento"],
    "Tempo (s)",
    "Valor",
    "Input do Controle de Direção vs Saída de Esterçamento",
    "input_vs_esterçamento.png"
)


# ============================================================
# POSIÇÃO DAS JUNTAS
# ============================================================

gerar_grafico(
    tempo,
    [
        pos_p1_log,
        pos_p2_log,
        pos_p3_log,
        pos_tool_log
    ],
    [
        "P1",
        "P2",
        "P3",
        "Tool"
    ],
    "Tempo (s)",
    "Posição Angular (rad)",
    "Posição das Juntas ao Longo do Tempo",
    "posicao_juntas.png"
)


# ============================================================
# INPUT VS P1
# ============================================================

gerar_grafico(
    tempo,
    [input_x_log, pos_p1_log],
    ["Input P1", "Posição P1"],
    "Tempo (s)",
    "Valor",
    "Input do Controle vs Posição da Junta P1",
    "input_vs_p1.png"
)


# ============================================================
# INPUT VS P2
# ============================================================

gerar_grafico(
    tempo,
    [input_y_log, pos_p2_log],
    ["Input P2", "Posição P2"],
    "Tempo (s)",
    "Valor",
    "Input do Controle vs Posição da Junta P2",
    "input_vs_p2.png"
)


# ============================================================
# INPUT VS P3
# ============================================================

gerar_grafico(
    tempo,
    [input_y2_log, pos_p3_log],
    ["Input P3", "Posição P3"],
    "Tempo (s)",
    "Valor",
    "Input do Controle vs Posição da Junta P3",
    "input_vs_p3.png"
)


# ============================================================
# INPUT VS TOOL
# ============================================================

gerar_grafico(
    tempo,
    [input_rt_log, pos_tool_log],
    ["Input Tool", "Posição Tool"],
    "Tempo (s)",
    "Valor",
    "Input do Controle vs Posição da Ferramenta",
    "input_vs_tool.png"
)


print(f"Gráficos salvos na pasta: {pasta_graficos}")