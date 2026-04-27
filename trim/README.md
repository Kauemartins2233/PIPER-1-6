# Aircraft Trim (Equilibrium) Module

## Visão geral

Este módulo implementa um algoritmo de trimagem (cálculo do ponto de equilíbrio) para um modelo não linear 6-DOF de aeronave, com o objetivo de determinar os estados iniciais e entradas de controle que resultam em um regime de voo estacionário.

O ponto de equilíbrio considerado neste repositório corresponde ao voo com asas niveladas, no qual as acelerações translacionais e rotacionais do corpo da aeronave são aproximadamente nulas.

O algoritmo foi desenvolvido para ser utilizado como etapa fundamental em:
- Inicialização de simulações não lineares
- Linearização do modelo em torno de um ponto de operação
- Projeto e validação de leis de controle
- Análises de estabilidade e desempenho

## Definição do problema de trimagem

Em termos de dinâmica de voo, o problema de trimagem consiste em encontrar um vetor de estados e controles tal que:

$\dot{x} = f(x, u) \approx 0$

onde:
- x representa o vetor de estados da aeronave (posição, velocidade, atitude, taxas angulares, etc.)
- u representa o vetor de entradas de controle (superfícies aerodinâmicas, empuxo, etc.)

Na prática, devido a não linearidades aerodinâmicas e acoplamentos dinâmicos, esse problema não admite solução analítica para modelos realistas, sendo necessário o uso de métodos numéricos.

## Abordagem adotada

### Formulação como um problema de otimização

O algoritmo de trimagem foi formulado como um problema de otimização não linear, resolvido utilizando o **fmincon**, do MATLAB.

A ideia central é transformar o problema de equilíbrio dinâmico em um problema de minimização da norma das acelerações do modelo:
  - A aeronave é simulada em um modelo 6-DOF não linear em Simulink
  - Um vetor candidato de trimagem é utilizado para:
    - Definir os estados iniciais
    - Definir o vetor de entradas de controle
  - O modelo é executado por um curto intervalo de tempo (0 segundos, apenas o instante inicial)
  - As acelerações translacionais e rotacionais resultantes são extraídas
  - Essas acelerações são utilizadas para compor uma função custo J

### Funcionamento do algoritmo de trimagem

O fluxo lógico do algoritmo é o seguinte:
1. Definição do vetor de trimagem (estados selecionados + entradas de controle)
2. Chute inicial fisicamente plausível fornecido ao fmincon
3. Execução do modelo não linear a cada iteração do fmincon
4. Cálculo da função custo a partir das acelerações
5. Processo iterativo até função custo abaixo do limiar
6. Validação do ponto de trimagem final

### Condição de voo considerada

Neste trabalho, o estado de equilíbrio implementado corresponde a:
- Voo com asas niveladas
- Taxas angulares próximas de zero
- Acelerações lineares e angulares residuais mínimas

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `trimagem_piper.m` | Script principal de trimagem via fmincon |
| `pipertrimcostfunction.m` | Função custo para o fmincon |
| `executar.m` | Script de inicialização (carrega .mat e addpath) |
| `q2_trim_modelo24a.slx` | Modelo Simulink usado na otimização de trim |

**Nota:** As funções compartilhadas (`equilibrium.m`, `dyn_rigidbody.m`, `aerodynamics.m`, `gerar_log.m`, etc.) ficam em `modelos/Não Linear/` e são adicionadas ao path automaticamente pelo `executar.m`.

## Como rodar

1. Rodar `executar.m` (carrega dados e adiciona path das funções compartilhadas)
2. Rodar `trimagem_piper.m` para executar a trimagem

```matlab
>> cd trim/
>> executar
>> trimagem_piper
```

## Dependências

- MATLAB com Optimization Toolbox (fmincon)
- Simulink
- Funções em `modelos/Não Linear/` (adicionadas ao path via `executar.m`)
