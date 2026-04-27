# Controle Não Linear - Piper J-3 Cub 1/6

Modelo Simulink (`modeloNL1.slx`) que aplica as malhas de controle longitudinal e látero-direcional sobre o **modelo não linear (6DOF)** da aeronave.

## Descrição

Neste modelo, a planta é o modelo não linear completo do Piper 1/6 (S-Function `sfunction_piper` com 21 outputs, mesma usada no NL_guidance.slx). As mesmas malhas PID projetadas sobre o modelo linear são aplicadas aqui para verificar se o controlador mantém desempenho aceitável quando a planta possui não linearidades, acoplamento entre modos e efeitos não modelados.

Os resultados mostraram que, para manobras suaves, as respostas do modelo não linear são praticamente sobrepostas às do modelo linear, validando o projeto dos controladores.

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `modeloNL1.slx` | Modelo Simulink com planta não linear (S-Function) + malhas de controle |

## Como rodar

1. Na raiz do repositório, rodar `inicializar` (carrega parâmetros, ganhos e equilíbrio).
2. Abrir `controle/Não Linear/modeloNL1.slx` no Simulink.
3. Definir as referências desejadas (velocidade, altitude, heading).
4. Simular (Ctrl+T).

```matlab
>> inicializar
>> open('controle/Não Linear/modeloNL1.slx')
>> % Simular (Ctrl+T)
```

## Dependências

Todas carregadas por `inicializar.m` (na raiz do repositório):
- `par_aero`, `par_prop`, `par_gen` (structs de parâmetros da aeronave)
- `Xe` (vetor de estado de equilíbrio, 12x1)
- `INPUTS` / `TrimInput` (vetor de comandos de equilíbrio)
- `C_alt`, `C_theta`, `C_vel`, `C_phi` (structs com ganhos PID)
- `Kq`, `Kp`, `Kp_sas`, `Kr` (ganhos dos SAS e amortecedores)

## Ponto de equilíbrio

```
Xe = [15; 0; -1.8343; 0; 0; 0; 0; -0.1217; 0; 0; 0; -100]
Ue = [0.4914; 0.0155; 0; 0]
```

## Ganhos dos controladores

Os ganhos são carregados do workspace via `inicializar.m`. São os mesmos valores usados no NL_guidance.slx.

**Nota sobre velocidade:** O PID de velocidade no modeloNL1 usa ganhos com sinal negativo (`-C_vel.Kp`, `-C_vel.Ki`, `-C_vel.Kd`) devido à topologia interna do autopilot. Os valores absolutos são os mesmos do NL_guidance.

### Longitudinal

| Malha | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|------|---|---|---|---|-----------|-------------|
| Altitude Hold | PID | 0.08 | 0.02 | 0.0 | 20 | [-0.17, 0.26] | clamping |
| Pitch (Atitude) | PID | 0.260 | 0.143 | 0.0 | 20 | [-0.4363, 0.4363] | clamping |
| SAS Arfagem | Ganho | Kq = 0.1 | | | | | |
| Velocidade | PID | 0.05 | 0.02 | 0.01 | 20 | [-0.49, 0.51] | clamping |

### Látero-direcional

| Malha | Tipo | P | I | D | N | Saturação | Anti-Windup |
|-------|------|---|---|---|---|-----------|-------------|
| Roll (Bank Angle) | PID | 10.0 | 0.0 | 0.0 | 20 | [-0.43, 0.43] | back-calculation |
| SAS Rolamento | Ganho | Kp = 0.119 | | | | | |
| Heading → phi | Ganho | 0.3 | | | | | |
| Amortecedor Guinada | Ganho + Washout | Kr = 0.15, filtro s/(s+1) | | | | | |

### Saturações dos atuadores

| Atuador | Limites | Unidade |
|---------|---------|---------|
| Sat_Throttle | [0, 1] | adimensional |
| Sat_Elevator | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Aileron | [-0.4363, 0.4363] | rad (±25°) |
| Sat_Rudder | [-0.4363, 0.4363] | rad (±25°) |

Ver diagramas de blocos completos em `controle/readme.md`.

## Resultados

### Longitudinal
- Altitude: erro máximo < 0.03 m em relação à referência
- Velocidade (VT): estabiliza próximo a 15 m/s
- Theta: estabiliza em valor ligeiramente acima do trim após transiente

### Látero-direcional
- Phi e Psi: respostas praticamente idênticas ao modelo linear
- Curva coordenada: excursão transitória em phi gera taxa de giro, aeronave nivela e estabiliza na nova proa
- Erro estático desprezível em ambas as variáveis
