# Aircraft Trim (Equilibrium) Module

## Visao geral

Este modulo implementa um algoritmo de trimagem (calculo do ponto de equilibrio) para um modelo nao linear 6-DOF de aeronave, com o objetivo de determinar os estados iniciais e entradas de controle que resultam em um regime de voo estacionario.

O ponto de equilibrio considerado neste repositorio corresponde ao voo com asas niveladas, no qual as aceleracoes translacionais e rotacionais do corpo da aeronave sao aproximadamente nulas.

O algoritmo foi desenvolvido para ser utilizado como etapa fundamental em:
- Inicializacao de simulacoes nao lineares
- Linearizacao do modelo em torno de um ponto de operacao
- Projeto e validacao de leis de controle
- Analises de estabilidade e desempenho

## Definicao do problema de trimagem

Em termos de dinamica de voo, o problema de trimagem consiste em encontrar um vetor de estados e controles tal que:

$\dot{x} = f(x, u) \approx 0$

onde:
- x representa o vetor de estados da aeronave (posicao, velocidade, atitude, taxas angulares, etc.)
- u representa o vetor de entradas de controle (superficies aerodinamicas, empuxo, etc.)

Na pratica, devido a nao linearidades aerodinamicas e acoplamentos dinamicos, esse problema nao admite solucao analitica para modelos realistas, sendo necessario o uso de metodos numericos.

## Abordagem adotada

### Formulacao como um problema de otimizacao

O algoritmo de trimagem foi formulado como um problema de otimizacao nao linear, resolvido utilizando o **fmincon**, do MATLAB.

A ideia central e transformar o problema de equilibrio dinamico em um problema de minimizacao da norma das aceleracoes do modelo:
  - A aeronave e simulada em um modelo 6-DOF nao linear em Simulink
  - Um vetor candidato de trimagem e utilizado para:
    - Definir os estados iniciais
    - Definir o vetor de entradas de controle
  - O modelo e executado por um curto intervalo de tempo (0 segundos, apenas o instante inicial)
  - As aceleracoes translacionais e rotacionais resultantes sao extraidas
  - Essas aceleracoes sao utilizadas para compor uma funcao custo J

### Funcionamento do algoritmo de trimagem

O fluxo logico do algoritmo e o seguinte:
1. Definicao do vetor de trimagem (estados selecionados + entradas de controle)
2. Chute inicial fisicamente plausivel fornecido ao fmincon
3. Execucao do modelo nao linear a cada iteracao do fmincon
4. Calculo da funcao custo a partir das aceleracoes
5. Processo iterativo ate funcao custo abaixo do limiar
6. Validacao do ponto de trimagem final

### Condicao de voo considerada

Neste trabalho, o estado de equilibrio implementado corresponde a:
- Voo com asas niveladas
- Taxas angulares proximas de zero
- Aceleracoes lineares e angulares residuais minimas

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `trimagem_piper.m` | Script principal de trimagem via fmincon |
| `pipertrimcostfunction.m` | Funcao custo para o fmincon |
| `executar.m` | Script de inicializacao (carrega .mat e addpath) |
| `q2_trim_modelo24a.slx` | Modelo Simulink usado na otimizacao de trim |

**Nota:** As funcoes compartilhadas (`equilibrium.m`, `dyn_rigidbody.m`, `aerodynamics.m`, `gerar_log.m`, etc.) ficam em `modelos/Nao Linear/` e sao adicionadas ao path automaticamente pelo `executar.m`.

## Como rodar

1. Rodar `executar.m` (carrega dados e adiciona path das funcoes compartilhadas)
2. Rodar `trimagem_piper.m` para executar a trimagem

```matlab
>> cd trim/
>> executar
>> trimagem_piper
```

## Dependencias

- MATLAB com Optimization Toolbox (fmincon)
- Simulink
- Funcoes em `modelos/Nao Linear/` (adicionadas ao path via `executar.m`)
