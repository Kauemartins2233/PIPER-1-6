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

O problema resolvido pelo fmincon pode ser interpretado como:  

  - Encontrar o vetor de estados e controles que minimiza as acelerações do modelo, respeitando eventuais restrições físicas (de acordo com a definição clássica de otimização).

### Funcionamento do algoritmo de trimagem

O fluxo lógico do algoritmo é o seguinte:
1. Definição do vetor de trimagem  
	•	O vetor de otimização é composto por estados selecionados e entradas de controle relevantes para o voo com asas niveladas.  
2.	Chute inicial  
	•	Um chute inicial fisicamente plausível é fornecido ao fmincon.  
	•	A qualidade desse chute influencia diretamente a convergência.  
3.	Execução do modelo não linear  
	•	A cada iteração, o fmincon atualiza o vetor de trimagem.  
	•	O modelo 6-DOF em Simulink é executado com esses valores.  
4.	Cálculo da função custo  
	•	As acelerações lineares e angulares do corpo da aeronave são coletadas.  
	•	A função custo é construída a partir dessas acelerações (tipicamente norma quadrática).  
5.	Processo iterativo  
	•	O fmincon ajusta o vetor de trimagem de modo a reduzir a função custo.  
	•	O processo continua até que a função custo fique abaixo de um limiar de sucesso pré-definido.  
6.	Validação do ponto de trimagem  
	•	O vetor final de trimagem é salvo.  
	•	A simulação é executada novamente com esse vetor.  
	•	As acelerações finais são plotadas para verificar que o equilíbrio foi atingido.  

### Condição de voo considerada

Neste trabalho, o estado de equilíbrio implementado corresponde a:  
	•	Voo com asas niveladas  
	•	Taxas angulares próximas de zero  
	•	Acelerações lineares e angulares residuais mínimas  

A estrutura do algoritmo, entretanto, é genérica e pode ser estendida para:  
	•	Voo em curva coordenada  
	•	Subida ou descida estabilizada  
	•	Outros pontos de operação específicos  

Observações importantes  
	•	A convergência do algoritmo depende fortemente:  
	  •	Do chute inicial  
	  •	Da escolha dos estados e controles incluídos no vetor de trimagem  
	  •	Da definição da função custo  
	•	O uso de otimização numérica torna o método robusto para modelos não lineares complexos, porém com custo computacional maior do que abordagens analíticas simplificadas.  
	•	O método é especialmente adequado para modelos de aeronaves utilizados em simulações de controle e dinâmica de voo.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `sfunction_piper_trim.m` | S-Function com 21 outputs (versao especifica para trimagem) |
| `trimagem_piper.m` | Script principal de trimagem via fmincon |
| `pipertrimcostfunction.m` | Funcao custo para o fmincon |
| `equilibrium.m` | Condicao de equilibrio (Xe, Ue) |
| `executar.m` | Script de inicializacao (carrega .mat e addpath) |
| `gerar_log.m` | Extrai variaveis latero-direcionais |
| `q2_trim_modelo24a.slx` | Modelo Simulink usado na otimizacao de trim |

## Dependencias

As funcoes de dinamica (`dyn_rigidbody.m`, `aerodynamics.m`, `propulsion.m`, etc.) ficam em `modelos/Não Linear/` e sao adicionadas ao path automaticamente pelo `executar.m`.

## Como rodar

1. Rodar `executar.m` (carrega dados e adiciona path das funcoes compartilhadas)
2. Rodar `trimagem_piper.m` para executar a trimagem

**Nota:** O bloco S-Function no modelo `q2_trim_modelo24a.slx` deve referenciar `sfunction_piper_trim` (nao `sfunction_piper`). Se necessario, atualize o nome no bloco S-Function do Simulink.

