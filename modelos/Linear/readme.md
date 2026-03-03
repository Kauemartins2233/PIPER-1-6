# Modelo Linear - Piper J-3 Cub 1/4

Matrizes do espaco de estados (A, B, C, D) dos modelos lineares longitudinal e latero-direcional da aeronave Piper J-3 Cub 1/4.

## Contexto

Tres metodologias de linearizacao foram avaliadas: script da Ana, metodo do livro do Nelson e funcao `linearize` do MATLAB. A comparacao com o modelo nao linear (via RMSE) indicou que os melhores modelos sao:

- **Longitudinal:** obtido via comando `linearize`
- **Latero-direcional:** obtido via script da Ana

As matrizes presentes em `MATRIZES.m` correspondem a esses dois modelos escolhidos.

## Arquivos

| Arquivo | Descricao |
|---------|-----------|
| `MATRIZES.m` | Define as matrizes A, B, C, D (longitudinal e latero-direcional) e os pontos de equilibrio |

## Matrizes do espaco de estados

### Longitudinal (via `linearize`)

Estados: `[U, W, q, theta, h]`
Entradas: `[delta_T, delta_e]`

```
A_long = [-0.7475 -0.2987  0.87   -9.734  -0.0001677;
          -2.005  -5.782   9.65    1.19   -0.0009366;
          -0.2089 -1.708  -20.32   0      -1.166e-10;
           0       0       1       0       0;
           0.1214  0.9926  0      -15.11   0]

B_long = [5.62 0; 0 0; 0 163.1; 0 0; 0 0]

C_long = diag([1 1 1 1 -1])    D_long = zeros(5,2)
```

### Latero-direcional (via script da Ana)

Estados: `[V, p, r, phi, psi]`
Entradas: `[delta_a, delta_r]`

```
A_lat = [-0.3414 -1.658  -14.75  9.734  0;
         -1.298  -28.3    7.924  0      0;
          0.9711 -2.081  -1.556  0      0;
          0       1      -0.1223 0      0;
          0       0       1.007  0      0]

B_lat = [0 0.0378; 3.296 0.0085; 0.171 -0.2389; 0 0; 0 0]

C_lat(1,1) = 0.0662, resto diagonal    D_lat = zeros(5,2)
```

## Pontos de equilibrio

Usados como condicao inicial nos blocos de espaco de estados do Simulink (devem coincidir com os do modelo nao linear):

```
Xe_long = [15, -1.8343, 0, -0.1217, -100]   % [U, W, q, theta, h]
Xe_lat  = [0, 0, 0, 0, 0]                    % [V, p, r, phi, psi]
```

## Passo a passo

1. Rodar `MATRIZES.m` para gerar as matrizes e pontos de equilibrio no workspace do MATLAB.
2. Abrir o modelo Simulink (`linear_models.slx`) que contem os blocos de espaco de estados. Esses blocos leem automaticamente as matrizes do workspace.
3. Definir a manobra desejada alterando os inputs no Simulink.
4. Adicionar os valores de `Xe_long` e `Xe_lat` como pontos iniciais na saida de cada bloco de espaco de estados.
5. Rodar a simulacao.
