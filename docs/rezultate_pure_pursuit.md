# Rezultate pure pursuit (pentru licență)

Simulator: f1tenth_gym_ros (dev-jazzy) + f1tenth_gym (dev-jax). Un tur, timp raportat de simulator („completed lap”).
Atingeri = numărul de mesaje `hit something` (mai multe mesaje pot veni de la același contact).

## Spielberg, `Spielberg_centerline.csv`

### Viteză constantă, lookahead fix
| viteză [m/s] | lookahead [m] | tur [s] | atingeri |
|---|---|---|---|
| 4 | 2.0 | 86 | 0 |
| 5 | 2.0 | 69.7 | 1 |
| 6 | 2.5 | 59.2 | 6 |
| 8 | 3.0 | 46.2 | 31 |

### Profil de viteză (curbură → a_lat, frânare înapoi, accelerare înainte)
vmin = 4 în toate rulările.

| vmax | a_lat | a_brake | a_acc | lookahead | tur [s] | atingeri | obs. |
|---|---|---|---|---|---|---|---|
| 8 | 6 | 6 | – | fix 2.0 | 47.0 | 0 | fără accel_profile |
| 10 | 8 | 6 | – | fix 2.0 | ~40 | 1 | ieșire largă după curba de la ~197 m |
| 10 | 8 | 6 | 3 | fix 2.0 | 40.0 | ? | |
| 15 | 8 | 6 | 3 | fix 2.0 | – | da | pierde controlul la frânare |
| 10 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | 42 | ? | |
| 12 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | **39.8** | **0** | cel mai bun |
| 12 | 8 | 3 | 3 | k=0.25, [1.5, 4.0] | 41.8 | 0 | |
| 12 | 8 | 6 | 5 | k=0.25, [1.5, 4.0] | 38.3 | 3 | |
| 12 | 8 | 6 | 4 | k=0.25, [1.5, 4.0] | 38.3 | 3 | atingeri după curba de la ~197 m (wp 481, 486 stânga; 508 dreapta) |
| 12 | 8 | 6 | 4 | k=0.3, [1.5, 4.0] | 39.2 | 6 | mai rău: taie curbele, atingeri în 3 locuri |

### Raceline
Raceline-ul TUM original trece la 0.17 m de perete (mediana 0.41 m), deci mașina (lățime 0.31 m) se lovește aproape peste tot.
Variante cu marjă generate cu `tools/make_safe_raceline.py` (marjă reală minimă: 0.31 / 0.42 / 0.50 m pentru 0.4 / 0.5 / 0.6).

| traseu | vmax | a_lat | a_brake | a_acc | lookahead | tur [s] | atingeri | obs. |
|---|---|---|---|---|---|---|---|---|
| raceline original | 12 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | – | multe | taie curbele, lovește peste tot |
| safe 0.5 | 12 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | 37.9 | 9 | 3 la start (pornește la 0.6 m de traseu), restul: erori de urmărire ~0.4–0.6 m, atât interior cât și exterior |
| safe 0.5, start pe traseu (14.76, 3.35, −2.851) | 12 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | 37.8 | 6 | fără atingeri la start; aceleași 6 locuri (s ≈ 238, 253, 261, 311, 317, 335 m) |
| safe 0.6, start pe traseu | 12 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | 41.7 | 9 | toate în penultima curbă (~(17–22, 19–24)); rămâne cu botul în perete, de aici timpul slab. Celelalte 5 zone au dispărut |
| safe 0.6, start pe traseu | 12 | 7 | 6 | 3 | k=0.25, [1.5, 4.0] | 38.3 | 1 | o atingere în penultima curbă (16.43, 24.56): taie interiorul cu ~0.3 m, colțul din față lovește |
| safe 0.6, start pe traseu | 12 | 7 | 6 | 3 | k=0.2, [1.5, 4.0] | ? | ? | șerpuiește pe dreptele lungi, după curbe |
| safe 0.6, start pe traseu | 12 | 7 | 6 | 3 | k=0.3, L0=−0.6, [1.5, 4.0] | 38.3 | 1 | aceeași atingere în penultima curbă (16.47, 24.54) |
| safe 0.6, start pe traseu | 12 | 7 | 4 | 3 | k=0.3, L0=−0.6, [1.5, 4.0] | 39.2 | 1 | penultima curbă rezolvată; atingere nouă la (−71.15, 45.41), pe dreapta de frânare dinaintea celei mai strânse curbe (κ 0.88), 0.39 m spre exterior |
| safe 0.6, start pe traseu | 12 | 7 | 3 | 3 | k=0.3, L0=−0.6, [1.5, 4.0] | **40.1** | **0** | primul tur curat pe raceline |
| safe 0.6, start pe traseu | 12 | 7 | 3 | 4 | k=0.3, L0=−0.6, [1.5, 4.0] | 39.2 | 0 | |
| safe 0.6, start pe traseu | 12 | 7.5 | 3 | 4 | k=0.3, L0=−0.6, [1.5, 4.0] | **38.8** | **0** | cel mai bun tur curat de până acum |
| safe 0.6, start pe traseu | 12 | 8 | 3 | 4 | k=0.3, L0=−0.6, [1.5, 4.0] | 38.35 | 1 | alunecă la ieșirea din ultima curbă, înainte de linia de start (13.39, 2.62) |
| safe 0.6, start pe traseu | 13 | 7.5 | 3 | 4 | k=0.3, L0=−0.6, [1.5, 4.0] | 40.2 | 7 | șerpuiește pe drepte la 13 m/s; atingeri în 2 zone |
| safe 0.6, start pe traseu | 12 | 7.5 | 3 | 5 | k=0.3, L0=−0.6, [1.5, 4.0] | 38.6 | 1 | atinge la ieșirea din ultima curbă (15.56, 3.21) |

**Configurația finală (în `spielberg_launch.py`):** safe 0.6, vmax 12, vmin 4, a_lat 7.5, a_brake 3, a_acc 4, k 0.3, L0 −0.6, L ∈ [1.5, 4.0] → **38.8 s, 0 atingeri**
(față de 86 s la 4 m/s constant: −55%). Start în `sim.yaml`: (14.76, 3.35, −2.851).

Limite găsite (peste ele apar atingeri): vmax 13 → șerpuiește pe drepte; a_lat 8 → alunecă la ieșirea din ultima curbă;
a_brake 4 → instabil la frânarea dinaintea celei mai strânse curbe; a_acc 5 → atinge la ieșirea din ultima curbă; marjă 0.5 → taie interiorul.
| 15 | 8 | 6 | 3 | k=0.25, [1.5, 4.0] | 51.1 | 14 | se învârte la frânarea pe drept (wp ~200) |
| 15 | 8 | 4 | 3 | k=0.25, [1.5, 4.0] | 41.1 | 5 | |
| 15 | 8 | 3 | 3 | k=0.25, [1.5, 4.0] | 41.4 | 0 | stabil, dar mai lent decât vmax 12 |

Observații:
- Frânarea reală în sim (~8.7 m/s²) e mai mare decât a_brake planificat: regulatorul de viteză frânează la maxim când viteza cerută scade.
  La 15 m/s, frânarea pe drept destabilizează mașina (eroarea de orientare crește de la 0.14 la 0.6 rad în 0.3 s).
- Frânarea mai blândă stabilizează 15 m/s, dar costă mai mult timp decât câștigă pe drept.

## levine_blocked
| variantă | tur [s] | atingeri |
|---|---|---|
| pure pursuit 3 m/s, L 1.5 | 20.7 | 0 |
| pure pursuit 4 m/s, L 1.5 | 15.7 | 0 |
| gap_follow (Lab 4) 6 m/s | ~11.0 | 0 |
